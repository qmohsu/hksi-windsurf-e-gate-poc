"""
Gate Geometry and Crossing Detection.

Computes perpendicular distance to start line and detects crossing events
for race timing.

Reference: Design Doc Section 8 (Gate Metrics)
"""

from typing import Optional, Tuple, Dict
from dataclasses import dataclass
import numpy as np

from bah_core.proto import (
    PositionEstimate,
    GateMetrics,
    CrossingEvent,
    create_no_gate_metrics,
)
from bah_core.localization import AnchorState
from bah_core.metrics import get_metrics


@dataclass
class GateConfig:
    """
    Configuration for gate calculator.
    
    Attributes:
        anchor_left_id: Anchor ID for left endpoint of gate
        anchor_right_id: Anchor ID for right endpoint of gate
        min_gate_length_m: Minimum valid gate length (m)
        max_gate_length_m: Maximum valid gate length (m)
        crossing_threshold_m: Threshold for crossing detection (m)
    """
    
    anchor_left_id: str = "A0"
    anchor_right_id: str = "A1"
    min_gate_length_m: float = 5.0   # Gates shorter than 5m are suspicious
    max_gate_length_m: float = 100.0  # Gates longer than 100m are suspicious
    crossing_threshold_m: float = 0.5  # Within 50cm of line for crossing


class GateCalculator:
    """
    Calculate gate metrics and detect crossing events.
    
    Usage:
        config = GateConfig(anchor_left_id="A0", anchor_right_id="A1")
        calc = GateCalculator(config)
        
        # Get anchor states
        anchor_states = {...}  # Dict[str, AnchorState]
        
        # Get position estimate
        estimate = position_pipeline.process(...)
        
        # Compute gate metrics
        gate_metrics = calc.compute_metrics(estimate, anchor_states)
        
        if gate_metrics.has_crossing:
            print(f"Crossing detected! Event: {gate_metrics.crossing_event}")
    
    Features:
    - Perpendicular distance calculation (d_perp_signed)
    - Along-line coordinate (s_along)
    - Crossing detection (sign change tracking)
    - Confidence scoring based on uncertainties
    """
    
    def __init__(self, config: Optional[GateConfig] = None):
        """
        Initialize gate calculator.
        
        Args:
            config: Gate configuration (uses defaults if None)
        """
        self.config = config or GateConfig()
        self.metrics_collector = get_metrics()
        
        # State for crossing detection
        self._last_d_perp_signed: Dict[str, float] = {}  # Per tag
        self._last_t_solve: Dict[str, float] = {}  # Per tag
    
    def compute_metrics(
        self,
        position_estimate: PositionEstimate,
        anchor_states: Dict[str, AnchorState]
    ) -> GateMetrics:
        """
        Compute gate metrics for a position estimate.
        
        Args:
            position_estimate: Tag position estimate from positioning pipeline
            anchor_states: Anchor states at solve time
            
        Returns:
            GateMetrics with d_perp_signed, s_along, crossing detection
            
        Notes:
            - If position has NO_FIX, returns placeholder metrics
            - If gate anchors not available, returns placeholder
            - Tracks crossing events per tag
        """
        tag_id = position_estimate.tag_id
        t_solve = position_estimate.t_solve
        
        # Check if position is valid
        if not position_estimate.has_valid_fix:
            self.metrics_collector.increment_drop('gate_no_fix')
            return create_no_gate_metrics(tag_id, t_solve)
        
        # Get gate anchor states
        anchor_left = anchor_states.get(self.config.anchor_left_id)
        anchor_right = anchor_states.get(self.config.anchor_right_id)
        
        if anchor_left is None or anchor_right is None:
            self.metrics_collector.increment_drop('gate_anchors_unavailable')
            return create_no_gate_metrics(tag_id, t_solve)
        
        # Extract positions
        tag_pos = np.array(position_estimate.pos_enu[:2])  # 2D (E, N)
        left_pos = np.array(anchor_left.pos_enu[:2])
        right_pos = np.array(anchor_right.pos_enu[:2])
        
        # Compute gate geometry
        gate_vector = right_pos - left_pos
        gate_length = float(np.linalg.norm(gate_vector))
        
        # Validate gate length
        if gate_length < self.config.min_gate_length_m:
            self.metrics_collector.increment_drop('gate_too_short')
            return create_no_gate_metrics(tag_id, t_solve)
        
        if gate_length > self.config.max_gate_length_m:
            self.metrics_collector.increment_drop('gate_too_long')
            return create_no_gate_metrics(tag_id, t_solve)
        
        # Compute perpendicular distance (signed)
        # Positive = right side of line (when looking from left to right)
        # Negative = left side of line
        d_perp_signed, s_along = self._compute_distance_to_line(
            tag_pos, left_pos, right_pos, gate_length
        )
        
        # Detect crossing
        crossing_event, crossing_confidence = self._detect_crossing(
            tag_id, t_solve, d_perp_signed, position_estimate, anchor_left, anchor_right
        )
        
        # Create metrics
        gate_metrics = GateMetrics(
            tag_id=tag_id,
            t_solve=t_solve,
            gate_anchor_left_id=self.config.anchor_left_id,
            gate_anchor_right_id=self.config.anchor_right_id,
            d_perp_signed=d_perp_signed,
            s_along=s_along,
            crossing_event=crossing_event,
            crossing_confidence=crossing_confidence,
            gate_length_m=gate_length,
            tag_position_quality=position_estimate.quality_score,
        )
        
        # Update metrics
        self.metrics_collector.increment('gate_metrics_computed')
        self.metrics_collector.record_histogram('gate_d_perp_m', abs(d_perp_signed))
        self.metrics_collector.record_histogram('gate_s_along', s_along)
        
        if crossing_event != CrossingEvent.NO_CROSSING:
            self.metrics_collector.increment('gate_crossings_detected')
        
        return gate_metrics
    
    def _compute_distance_to_line(
        self,
        point: np.ndarray,
        line_start: np.ndarray,
        line_end: np.ndarray,
        line_length: float
    ) -> Tuple[float, float]:
        """
        Compute signed perpendicular distance and along-line coordinate.
        
        Args:
            point: Tag position (2D)
            line_start: Left anchor position (2D)
            line_end: Right anchor position (2D)
            line_length: Precomputed line length
            
        Returns:
            (d_perp_signed, s_along)
            - d_perp_signed: Perpendicular distance (+ right, - left)
            - s_along: Coordinate along line (0 = start, 1 = end)
            
        Algorithm:
            1. Vector from line_start to point: v = point - line_start
            2. Unit vector along line: u = (line_end - line_start) / length
            3. Project v onto line: s_along = v · u / length
            4. Perpendicular component: v_perp = v - s_along * u
            5. Sign from cross product: sign = (v × u).z
        """
        line_vector = line_end - line_start
        line_unit = line_vector / line_length
        
        # Vector from line start to point
        v = point - line_start
        
        # Project onto line (dot product)
        s_along_distance = np.dot(v, line_unit)
        s_along = s_along_distance / line_length  # Normalized (0 = left, 1 = right)
        
        # Perpendicular component
        v_parallel = s_along_distance * line_unit
        v_perp = v - v_parallel
        
        # Perpendicular distance (unsigned)
        d_perp = float(np.linalg.norm(v_perp))
        
        # Sign from cross product (2D cross product gives z-component)
        # Positive = right side, negative = left side
        cross_z = line_unit[0] * v_perp[1] - line_unit[1] * v_perp[0]
        sign = 1.0 if cross_z >= 0 else -1.0
        
        d_perp_signed = sign * d_perp
        
        return float(d_perp_signed), float(s_along)
    
    def _detect_crossing(
        self,
        tag_id: str,
        t_solve: float,
        d_perp_signed: float,
        position_estimate: PositionEstimate,
        anchor_left: AnchorState,
        anchor_right: AnchorState
    ) -> Tuple[CrossingEvent, float]:
        """
        Detect crossing event based on sign change.
        
        Args:
            tag_id: Tag ID
            t_solve: Current time
            d_perp_signed: Current perpendicular distance
            position_estimate: Position estimate with uncertainties
            anchor_left: Left anchor state
            anchor_right: Right anchor state
            
        Returns:
            (crossing_event, crossing_confidence)
            
        Algorithm:
            1. Check if we have previous d_perp_signed for this tag
            2. If sign changed → crossing detected
            3. Confidence based on:
               - Position quality
               - Anchor uncertainties
               - Distance from line (closer = higher confidence)
        """
        # Check for previous state
        if tag_id not in self._last_d_perp_signed:
            # First measurement - no crossing yet
            self._last_d_perp_signed[tag_id] = d_perp_signed
            self._last_t_solve[tag_id] = t_solve
            return CrossingEvent.NO_CROSSING, 0.0
        
        last_d = self._last_d_perp_signed[tag_id]
        last_t = self._last_t_solve[tag_id]
        
        # Update state
        self._last_d_perp_signed[tag_id] = d_perp_signed
        self._last_t_solve[tag_id] = t_solve
        
        # Check for sign change
        if last_d * d_perp_signed >= 0:
            # Same sign or zero - no crossing
            return CrossingEvent.NO_CROSSING, 0.0
        
        # Sign changed - crossing detected!
        if last_d < 0 and d_perp_signed > 0:
            crossing_event = CrossingEvent.CROSSING_LEFT
        else:
            crossing_event = CrossingEvent.CROSSING_RIGHT
        
        # Compute confidence
        confidence = self._compute_crossing_confidence(
            position_estimate, anchor_left, anchor_right, d_perp_signed, last_d
        )
        
        return crossing_event, confidence
    
    def _compute_crossing_confidence(
        self,
        position_estimate: PositionEstimate,
        anchor_left: AnchorState,
        anchor_right: AnchorState,
        d_perp_now: float,
        d_perp_last: float
    ) -> float:
        """
        Compute confidence for crossing detection.
        
        Factors:
        1. Position quality (higher = more confident)
        2. Anchor uncertainties (lower = more confident)
        3. Proximity to line (closer = more confident)
        4. Jump magnitude (smaller = more confident)
        
        Args:
            position_estimate: Position estimate with quality
            anchor_left: Left anchor state with uncertainty
            anchor_right: Right anchor state with uncertainty
            d_perp_now: Current perpendicular distance
            d_perp_last: Previous perpendicular distance
            
        Returns:
            Confidence (0-1)
        """
        # Factor 1: Position quality
        pos_quality = position_estimate.quality_score or 0.5
        
        # Factor 2: Anchor uncertainty
        # Average position uncertainty of gate endpoints
        # pos_cov is 3x3 covariance, take diagonal for std devs
        if anchor_left.pos_cov is not None:
            left_unc = np.mean(np.sqrt(np.diag(anchor_left.pos_cov)[:2]))
        else:
            left_unc = 1.0
        
        if anchor_right.pos_cov is not None:
            right_unc = np.mean(np.sqrt(np.diag(anchor_right.pos_cov)[:2]))
        else:
            right_unc = 1.0
        
        avg_anchor_unc = (left_unc + right_unc) / 2.0
        
        # Convert to quality (smaller uncertainty = higher quality)
        anchor_quality = np.exp(-avg_anchor_unc / 1.0)  # 1m characteristic scale
        
        # Factor 3: Proximity to line
        # Closer to line at crossing = higher confidence
        avg_dist = (abs(d_perp_now) + abs(d_perp_last)) / 2.0
        proximity_quality = np.exp(-avg_dist / 2.0)  # 2m characteristic scale
        
        # Factor 4: Jump magnitude
        # Smaller jumps are more believable
        jump_mag = abs(d_perp_now - d_perp_last)
        jump_quality = np.exp(-jump_mag / 5.0)  # 5m characteristic scale
        
        # Combine factors (geometric mean)
        confidence = float(
            (pos_quality * anchor_quality * proximity_quality * jump_quality) ** 0.25
        )
        
        # Clamp to [0, 1]
        confidence = max(0.0, min(1.0, confidence))
        
        return confidence
    
    def reset_tag_history(self, tag_id: str):
        """Reset crossing detection history for a tag."""
        if tag_id in self._last_d_perp_signed:
            del self._last_d_perp_signed[tag_id]
        if tag_id in self._last_t_solve:
            del self._last_t_solve[tag_id]
        self.metrics_collector.increment('gate_history_resets')
    
    def reset_all_history(self):
        """Reset all crossing detection history."""
        self._last_d_perp_signed.clear()
        self._last_t_solve.clear()
        self.metrics_collector.increment('gate_history_resets')


def create_default_gate_calculator(
    anchor_left_id: str = "A0",
    anchor_right_id: str = "A1"
) -> GateCalculator:
    """
    Create gate calculator with default configuration.
    
    Args:
        anchor_left_id: Left anchor ID for gate
        anchor_right_id: Right anchor ID for gate
        
    Returns:
        Configured GateCalculator
    """
    config = GateConfig(
        anchor_left_id=anchor_left_id,
        anchor_right_id=anchor_right_id,
        min_gate_length_m=5.0,
        max_gate_length_m=100.0,
        crossing_threshold_m=0.5,
    )
    
    return GateCalculator(config)
