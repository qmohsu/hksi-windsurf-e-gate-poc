"""
Plausibility Checker for Tag Position Estimates.

Validates that multilateration solutions are physically plausible
based on maximum speed and acceleration constraints.

With only 3 anchors (no redundancy), plausibility checking is critical
to detect and reject outliers that would cause "teleporting".

Reference: Design Doc Section 7.3 (Solution plausibility checks)
"""

from typing import Optional
from dataclasses import dataclass
import numpy as np

from bah_core.proto.position_estimate import PositionEstimate
from bah_core.metrics import get_metrics


@dataclass
class PlausibilityConfig:
    """
    Configuration for plausibility checker.
    
    Attributes:
        max_speed_m_s: Maximum plausible speed (m/s)
        max_acceleration_m_s2: Maximum plausible acceleration (m/s²)
        max_innovation_m: Maximum innovation vs filter prediction (m)
        min_quality_after_fail: Quality score after plausibility failure
    """
    
    max_speed_m_s: float = 15.0        # Windsurfing: ~15 m/s max
    max_acceleration_m_s2: float = 5.0  # ~0.5g max
    max_innovation_m: float = 10.0      # Max 10m jump from prediction
    min_quality_after_fail: float = 0.1  # Heavily degrade quality


class PlausibilityChecker:
    """
    Check physical plausibility of tag position estimates.
    
    Usage:
        checker = PlausibilityChecker(config)
        
        # Check multilateration solution
        estimate = tag_solver.solve(...)
        
        # With kinematic filter prediction
        if filter.is_initialized():
            predicted_pos = filter.get_predicted_position(estimate.t_solve)
            is_plausible = checker.check_innovation(estimate, predicted_pos)
        
        # Without filter (check speed/acceleration only)
        is_plausible = checker.check_estimate(estimate, prev_estimate, dt)
        
        if not is_plausible:
            # Reject or degrade quality
            estimate.quality_score = config.min_quality_after_fail
    
    Features:
    - Speed validation: v = ||Δpos|| / Δt < v_max
    - Acceleration validation: a = ||Δvel|| / Δt < a_max
    - Innovation validation: ||pos - predicted|| < max_innovation
    - Metrics tracking for all failures
    """
    
    def __init__(self, config: Optional[PlausibilityConfig] = None):
        """
        Initialize plausibility checker.
        
        Args:
            config: Checker configuration (uses defaults if None)
        """
        self.config = config or PlausibilityConfig()
        self.metrics = get_metrics()
    
    def check_estimate(
        self,
        current: PositionEstimate,
        previous: Optional[PositionEstimate],
        dt: float
    ) -> bool:
        """
        Check plausibility of current estimate vs previous.
        
        Args:
            current: Current position estimate
            previous: Previous position estimate (can be None)
            dt: Time between estimates (s)
            
        Returns:
            True if plausible, False if failed checks
            
        Side Effects:
            - Updates metrics counters
        """
        if previous is None or dt <= 0:
            # No previous estimate or invalid dt: accept
            return True
        
        # Check speed
        if not self._check_speed(current, previous, dt):
            self.metrics.increment_drop('plausibility_speed_exceeded')
            return False
        
        # Check acceleration (if both have velocity)
        if current.vel_enu is not None and previous.vel_enu is not None:
            if not self._check_acceleration(current, previous, dt):
                self.metrics.increment_drop('plausibility_accel_exceeded')
                return False
        
        # All checks passed
        self.metrics.increment('plausibility_checks_passed')
        return True
    
    def check_innovation(
        self,
        measurement: PositionEstimate,
        predicted_pos: tuple
    ) -> bool:
        """
        Check innovation magnitude vs filter prediction.
        
        Args:
            measurement: Measured position estimate
            predicted_pos: Predicted position from filter (E, N, U)
            
        Returns:
            True if innovation within bounds, False otherwise
            
        Notes:
            - Innovation = ||measurement - prediction||
            - Large innovation suggests outlier measurement
        """
        meas_pos = np.array(measurement.pos_enu[:2])  # 2D (E, N)
        pred_pos = np.array(predicted_pos[:2])
        
        innovation = np.linalg.norm(meas_pos - pred_pos)
        
        if innovation > self.config.max_innovation_m:
            self.metrics.increment_drop('plausibility_innovation_exceeded')
            self.metrics.record_histogram('plausibility_innovation_m', innovation)
            return False
        
        self.metrics.record_histogram('plausibility_innovation_m', innovation)
        return True
    
    def degrade_quality(
        self,
        estimate: PositionEstimate,
        reason: str
    ) -> PositionEstimate:
        """
        Degrade quality score of estimate that failed plausibility.
        
        Args:
            estimate: Position estimate to degrade
            reason: Reason for degradation
            
        Returns:
            New PositionEstimate with degraded quality
        """
        degraded = PositionEstimate(
            tag_id=estimate.tag_id,
            t_solve=estimate.t_solve,
            fix_type=estimate.fix_type,
            pos_enu=estimate.pos_enu,
            quality_score=self.config.min_quality_after_fail,
            num_anchors_used=estimate.num_anchors_used,
            anchor_ids=estimate.anchor_ids,
            residual_m=estimate.residual_m,
            pos_std_enu=estimate.pos_std_enu,
            geometry_score=estimate.geometry_score,
            vel_enu=estimate.vel_enu,
            innovation_m=estimate.innovation_m,
        )
        
        self.metrics.increment(f'plausibility_degraded_{reason}')
        
        return degraded
    
    def _check_speed(
        self,
        current: PositionEstimate,
        previous: PositionEstimate,
        dt: float
    ) -> bool:
        """Check if implied speed is within bounds."""
        curr_pos = np.array(current.pos_enu[:2])
        prev_pos = np.array(previous.pos_enu[:2])
        
        displacement = np.linalg.norm(curr_pos - prev_pos)
        speed = displacement / dt
        
        if speed > self.config.max_speed_m_s:
            self.metrics.record_histogram('plausibility_speed_m_s', speed)
            return False
        
        return True
    
    def _check_acceleration(
        self,
        current: PositionEstimate,
        previous: PositionEstimate,
        dt: float
    ) -> bool:
        """Check if implied acceleration is within bounds."""
        curr_vel = np.array(current.vel_enu[:2])
        prev_vel = np.array(previous.vel_enu[:2])
        
        delta_vel = np.linalg.norm(curr_vel - prev_vel)
        acceleration = delta_vel / dt
        
        if acceleration > self.config.max_acceleration_m_s2:
            self.metrics.record_histogram('plausibility_accel_m_s2', acceleration)
            return False
        
        return True
    
    def get_statistics(self) -> dict:
        """Get plausibility checker statistics."""
        return {
            'checks_passed': self.metrics.get_counter('plausibility_checks_passed'),
            'speed_exceeded': self.metrics.get_counter('plausibility_speed_exceeded'),
            'accel_exceeded': self.metrics.get_counter('plausibility_accel_exceeded'),
            'innovation_exceeded': self.metrics.get_counter('plausibility_innovation_exceeded'),
        }


def create_default_checker() -> PlausibilityChecker:
    """
    Create plausibility checker with default configuration for windsurfing.
    
    Returns:
        Configured PlausibilityChecker instance
    """
    config = PlausibilityConfig(
        max_speed_m_s=15.0,        # ~30 knots max for windsurfing
        max_acceleration_m_s2=5.0,  # ~0.5g
        max_innovation_m=10.0,      # 10m max jump
        min_quality_after_fail=0.1,
    )
    
    return PlausibilityChecker(config)
