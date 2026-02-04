"""
Tag Kinematic Filter (Constant-Velocity).

Implements a simple 4D constant-velocity Kalman filter for tag position
smoothing and holdover during brief measurement dropouts.

State: [E, N, vE, vN] (2D position + velocity)

Reference: Design Doc Section 7.3 (Tag kinematic smoothing)
"""

from typing import Optional, Tuple
from dataclasses import dataclass
import numpy as np
import time

from bah_core.proto.position_estimate import PositionEstimate, FixType
from bah_core.metrics import get_metrics


@dataclass
class TagKinematicFilterConfig:
    """
    Configuration for tag kinematic filter.
    
    Attributes:
        q_pos: Process noise for position (m²/s)
        q_vel: Process noise for velocity (m²/s³)
        max_hold_time_s: Maximum time to hold without measurement (s)
        initial_pos_std_m: Initial position uncertainty (m)
        initial_vel_std_m_s: Initial velocity uncertainty (m/s)
    """
    
    q_pos: float = 0.1 ** 2      # Position process noise (10cm)
    q_vel: float = 1.0 ** 2      # Velocity process noise (1 m/s)
    max_hold_time_s: float = 2.0  # Max 2s holdover
    initial_pos_std_m: float = 10.0  # Initial position uncertainty
    initial_vel_std_m_s: float = 5.0  # Initial velocity uncertainty


class TagKinematicFilter:
    """
    Constant-velocity Kalman filter for tag position smoothing.
    
    Usage:
        filter = TagKinematicFilter("T1", config)
        
        # Update with multilateration solution
        estimate = tag_solver.solve(ranges, anchors)
        if estimate.has_valid_fix:
            filtered = filter.update(estimate)
        else:
            # Hold during dropout
            held = filter.hold(t_now)
        
        # Get smoothed position
        smoothed_pos = filtered.pos_enu
    
    Features:
    - Smooths jittery multilateration solutions
    - Provides velocity estimate
    - Can hold for brief periods without measurement
    - Computes innovation for plausibility checking
    """
    
    def __init__(self, tag_id: str, config: Optional[TagKinematicFilterConfig] = None):
        """
        Initialize kinematic filter.
        
        Args:
            tag_id: Tag ID
            config: Filter configuration (uses defaults if None)
        """
        self.tag_id = tag_id
        self.config = config or TagKinematicFilterConfig()
        self.metrics = get_metrics()
        
        # State: [E, N, vE, vN]
        self._state: Optional[np.ndarray] = None
        
        # Covariance: 4x4
        self._covariance: Optional[np.ndarray] = None
        
        # Last update time
        self._last_update_time: Optional[float] = None
        
        # Last measurement position (for hold)
        self._last_pos_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    def is_initialized(self) -> bool:
        """Check if filter has been initialized."""
        return self._state is not None
    
    def update(self, measurement: PositionEstimate) -> PositionEstimate:
        """
        Update filter with multilateration measurement.
        
        Args:
            measurement: PositionEstimate from TagSolver
            
        Returns:
            Filtered PositionEstimate with smoothed position and velocity
            
        Notes:
            - If not initialized, initializes from first measurement
            - Updates state and covariance with Kalman filter
            - Computes innovation for plausibility checking
        """
        if not measurement.has_valid_fix:
            # Don't update with NO_FIX
            return measurement
        
        t_now = measurement.t_solve
        
        # Initialize if needed
        if not self.is_initialized():
            self._initialize_from_measurement(measurement)
            
            # First measurement: return as-is but mark innovation
            measurement_copy = PositionEstimate(
                tag_id=measurement.tag_id,
                t_solve=measurement.t_solve,
                fix_type=measurement.fix_type,
                pos_enu=measurement.pos_enu,
                quality_score=measurement.quality_score,
                num_anchors_used=measurement.num_anchors_used,
                anchor_ids=measurement.anchor_ids,
                residual_m=measurement.residual_m,
                pos_std_enu=measurement.pos_std_enu,
                geometry_score=measurement.geometry_score,
                vel_enu=(0.0, 0.0, 0.0),  # Zero velocity initially
                innovation_m=0.0,  # No innovation for first measurement
            )
            
            self.metrics.increment('tag_filter_initialized')
            return measurement_copy
        
        # Predict forward
        dt = t_now - self._last_update_time
        self._predict(dt)
        
        # Measurement: [E, N]
        z = np.array([measurement.pos_enu[0], measurement.pos_enu[1]])
        
        # Measurement noise from estimate (or default)
        if measurement.pos_std_enu is not None:
            r_std = np.array([measurement.pos_std_enu[0], measurement.pos_std_enu[1]])
        else:
            # Default: use residual as proxy for uncertainty
            r_std = np.array([measurement.residual_m, measurement.residual_m])
        
        R = np.diag(r_std ** 2 + 0.01)  # Add small value for numerical stability
        
        # Update step
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        y = z - H @ self._state  # Innovation
        S = H @ self._covariance @ H.T + R
        K = self._covariance @ H.T @ np.linalg.inv(S)
        
        self._state = self._state + K @ y
        self._covariance = (np.eye(4) - K @ H) @ self._covariance
        
        self._last_update_time = t_now
        
        # Innovation magnitude (for plausibility checking)
        innovation_m = float(np.linalg.norm(y))
        
        # Create filtered estimate
        filtered_pos = (
            float(self._state[0]),
            float(self._state[1]),
            measurement.pos_enu[2]  # Keep Z from measurement
        )
        
        filtered_vel = (
            float(self._state[2]),
            float(self._state[3]),
            0.0  # No vertical velocity
        )
        
        # Position uncertainty from covariance
        pos_std = (
            float(np.sqrt(self._covariance[0, 0])),
            float(np.sqrt(self._covariance[1, 1])),
            measurement.pos_std_enu[2] if measurement.pos_std_enu else 1.0
        )
        
        filtered = PositionEstimate(
            tag_id=measurement.tag_id,
            t_solve=t_now,
            fix_type=measurement.fix_type,
            pos_enu=filtered_pos,
            quality_score=measurement.quality_score,  # Will be updated by plausibility checker
            num_anchors_used=measurement.num_anchors_used,
            anchor_ids=measurement.anchor_ids,
            residual_m=measurement.residual_m,
            pos_std_enu=pos_std,
            geometry_score=measurement.geometry_score,
            vel_enu=filtered_vel,
            innovation_m=innovation_m,
        )
        
        self._last_pos_enu = filtered_pos
        
        self.metrics.increment('tag_filter_updates')
        self.metrics.record_histogram('tag_filter_innovation_m', innovation_m)
        
        return filtered
    
    def hold(self, t_now: float) -> Optional[PositionEstimate]:
        """
        Hold position during measurement dropout.
        
        Args:
            t_now: Current time
            
        Returns:
            Held PositionEstimate, or None if hold time exceeded
            
        Notes:
            - Uses constant-velocity prediction
            - Quality degrades with hold time
            - Returns None if hold exceeds max_hold_time_s
        """
        if not self.is_initialized():
            return None
        
        dt = t_now - self._last_update_time
        
        # Check hold time limit
        if dt > self.config.max_hold_time_s:
            self.metrics.increment_drop('tag_filter_hold_exceeded')
            return None
        
        # Predict forward
        self._predict(dt)
        
        # Held position
        held_pos = (
            float(self._state[0]),
            float(self._state[1]),
            self._last_pos_enu[2]  # Keep last Z
        )
        
        held_vel = (
            float(self._state[2]),
            float(self._state[3]),
            0.0
        )
        
        # Position uncertainty (growing with time)
        pos_std = (
            float(np.sqrt(self._covariance[0, 0])),
            float(np.sqrt(self._covariance[1, 1])),
            1.0
        )
        
        # Quality degrades with hold time (exponential decay)
        # Quality = exp(-dt / characteristic_time)
        quality_degradation = np.exp(-dt / 1.0)  # 1s characteristic time
        held_quality = float(0.5 * quality_degradation)  # Start at 0.5 for HOLD
        
        held = PositionEstimate(
            tag_id=self.tag_id,
            t_solve=t_now,
            fix_type=FixType.HOLD,
            pos_enu=held_pos,
            quality_score=held_quality,
            num_anchors_used=0,
            anchor_ids=[],
            residual_m=0.0,
            pos_std_enu=pos_std,
            vel_enu=held_vel,
        )
        
        self.metrics.increment('tag_filter_holds')
        
        return held
    
    def _initialize_from_measurement(self, measurement: PositionEstimate):
        """Initialize filter from first measurement."""
        # Initial state: [E, N, 0, 0] (zero velocity)
        self._state = np.array([
            measurement.pos_enu[0],
            measurement.pos_enu[1],
            0.0,  # vE
            0.0,  # vN
        ])
        
        # Initial covariance
        self._covariance = np.diag([
            self.config.initial_pos_std_m ** 2,
            self.config.initial_pos_std_m ** 2,
            self.config.initial_vel_std_m_s ** 2,
            self.config.initial_vel_std_m_s ** 2,
        ])
        
        self._last_update_time = measurement.t_solve
        self._last_pos_enu = measurement.pos_enu
    
    def _predict(self, dt: float):
        """Predict state forward by dt seconds."""
        if dt <= 0:
            return
        
        # State transition matrix: constant velocity
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Process noise
        Q = np.array([
            [self.config.q_pos * dt, 0, 0, 0],
            [0, self.config.q_pos * dt, 0, 0],
            [0, 0, self.config.q_vel * dt, 0],
            [0, 0, 0, self.config.q_vel * dt]
        ])
        
        # Predict
        self._state = F @ self._state
        self._covariance = F @ self._covariance @ F.T + Q
    
    def get_predicted_position(self, t_predict: float) -> Optional[Tuple[float, float, float]]:
        """
        Get predicted position at future time.
        
        Args:
            t_predict: Time to predict to
            
        Returns:
            Predicted (E, N, U) position, or None if not initialized
        """
        if not self.is_initialized():
            return None
        
        dt = t_predict - self._last_update_time
        
        # Predict (without updating internal state)
        state_copy = self._state.copy()
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        predicted_state = F @ state_copy
        
        return (
            float(predicted_state[0]),
            float(predicted_state[1]),
            self._last_pos_enu[2]
        )
    
    def reset(self):
        """Reset filter to uninitialized state."""
        self._state = None
        self._covariance = None
        self._last_update_time = None
        self._last_pos_enu = (0.0, 0.0, 0.0)
        self.metrics.increment('tag_filter_resets')
