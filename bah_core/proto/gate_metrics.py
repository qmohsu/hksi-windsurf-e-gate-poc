"""
Gate Metrics Message Schema.

Defines the output format for start-line crossing detection and
perpendicular distance calculations.

Reference: Design Doc Section 8 (Gate Metrics)
"""

from dataclasses import dataclass
from typing import Optional
from enum import IntEnum


class CrossingEvent(IntEnum):
    """Type of crossing event detected."""
    NO_CROSSING = 0      # No crossing detected
    CROSSING_LEFT = 1    # Crossed from left side (negative) to right (positive)
    CROSSING_RIGHT = 2   # Crossed from right side (positive) to left (negative)


@dataclass
class GateMetrics:
    """
    Gate metrics for start-line crossing detection.
    
    Attributes:
        tag_id: Tag ID
        t_solve: Time of position estimate (same as PositionEstimate.t_solve)
        gate_anchor_left_id: Left anchor ID defining gate
        gate_anchor_right_id: Right anchor ID defining gate
        d_perp_signed: Signed perpendicular distance to gate line (m)
        s_along: Along-line coordinate (0 = left, 1 = right, can be outside)
        crossing_event: Crossing event type
        crossing_confidence: Crossing confidence (0-1)
        gate_length_m: Length of gate line (m)
        tag_position_quality: Quality from PositionEstimate
    """
    
    tag_id: str
    t_solve: float
    gate_anchor_left_id: str
    gate_anchor_right_id: str
    d_perp_signed: float
    s_along: float
    crossing_event: CrossingEvent = CrossingEvent.NO_CROSSING
    crossing_confidence: float = 0.0
    gate_length_m: Optional[float] = None
    tag_position_quality: Optional[float] = None
    
    @property
    def is_left_of_line(self) -> bool:
        """Check if tag is on left side of line."""
        return self.d_perp_signed < 0.0
    
    @property
    def is_right_of_line(self) -> bool:
        """Check if tag is on right side of line."""
        return self.d_perp_signed > 0.0
    
    @property
    def is_on_line(self) -> bool:
        """Check if tag is approximately on line (within 10cm)."""
        return abs(self.d_perp_signed) < 0.1
    
    @property
    def is_within_gate_bounds(self) -> bool:
        """Check if tag is within gate segment bounds (0 <= s_along <= 1)."""
        return 0.0 <= self.s_along <= 1.0
    
    @property
    def has_crossing(self) -> bool:
        """Check if a crossing event was detected."""
        return self.crossing_event != CrossingEvent.NO_CROSSING
    
    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            'tag_id': self.tag_id,
            't_solve': self.t_solve,
            'gate_anchor_left_id': self.gate_anchor_left_id,
            'gate_anchor_right_id': self.gate_anchor_right_id,
            'd_perp_signed': self.d_perp_signed,
            's_along': self.s_along,
            'crossing_event': self.crossing_event.name,
            'crossing_confidence': self.crossing_confidence,
            'gate_length_m': self.gate_length_m,
            'tag_position_quality': self.tag_position_quality,
        }


def create_no_gate_metrics(tag_id: str, t_solve: float) -> GateMetrics:
    """
    Create placeholder gate metrics when gate cannot be computed.
    
    Args:
        tag_id: Tag ID
        t_solve: Time
        
    Returns:
        GateMetrics with zero values
    """
    return GateMetrics(
        tag_id=tag_id,
        t_solve=t_solve,
        gate_anchor_left_id="",
        gate_anchor_right_id="",
        d_perp_signed=0.0,
        s_along=0.0,
        crossing_event=CrossingEvent.NO_CROSSING,
        crossing_confidence=0.0,
    )
