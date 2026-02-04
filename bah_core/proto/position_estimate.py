"""
Position Estimate Output Schema.

Defines the output format for tag position estimates from multilateration.
Used in P1-C for tag solver output.

Reference: Design Doc Section 7.4 (Quality Score)
"""

from dataclasses import dataclass
from typing import Optional, Tuple
from enum import IntEnum
import time


class FixType(IntEnum):
    """Type of position fix."""
    
    NO_FIX = 0          # No valid solution
    FIX_2D = 1          # 2D position (E, N) on water surface
    FIX_3D = 2          # 3D position (E, N, U)
    HOLD = 3            # Held position from kinematic filter


@dataclass
class PositionEstimate:
    """
    Tag position estimate from multilateration.
    
    Attributes:
        tag_id: ID of the tag
        t_solve: Time at which position was solved (t_range)
        fix_type: Type of fix (NO_FIX, FIX_2D, FIX_3D, HOLD)
        pos_enu: Position in ENU frame (E, N, U) in meters
        quality_score: Quality indicator (0-1, higher is better)
        num_anchors_used: Number of anchors used in solution
        anchor_ids: List of anchor IDs used
        residual_m: RMS residual from multilateration (m)
        
        # Optional uncertainty
        pos_std_enu: Position standard deviation (E, N, U) in meters
        
        # Optional geometry health
        geometry_score: Geometry health (0-1, based on triangle/DOP)
        
        # Optional kinematic info
        vel_enu: Velocity estimate (E, N, U) in m/s (if filter enabled)
        innovation_m: Innovation magnitude vs prediction (m)
        
    Notes:
        - pos_enu is always populated (even for NO_FIX, contains last valid/held)
        - For NO_FIX, quality_score = 0
        - For HOLD, quality_score degrades with hold time
    """
    
    tag_id: str
    t_solve: float
    fix_type: FixType
    pos_enu: Tuple[float, float, float]
    quality_score: float
    num_anchors_used: int
    anchor_ids: list
    residual_m: float
    
    # Optional fields
    pos_std_enu: Optional[Tuple[float, float, float]] = None
    geometry_score: Optional[float] = None
    vel_enu: Optional[Tuple[float, float, float]] = None
    innovation_m: Optional[float] = None
    
    def __post_init__(self):
        """Validate position estimate."""
        if not 0 <= self.quality_score <= 1:
            raise ValueError(f"Quality score must be in [0,1]: {self.quality_score}")
        
        if self.num_anchors_used < 0:
            raise ValueError(f"Num anchors cannot be negative: {self.num_anchors_used}")
        
        if self.residual_m < 0:
            raise ValueError(f"Residual cannot be negative: {self.residual_m}")
    
    @property
    def has_valid_fix(self) -> bool:
        """Check if this is a valid position fix (not NO_FIX)."""
        return self.fix_type != FixType.NO_FIX
    
    @property
    def is_2d_fix(self) -> bool:
        """Check if this is a 2D fix on water surface."""
        return self.fix_type == FixType.FIX_2D
    
    @property
    def is_3d_fix(self) -> bool:
        """Check if this is a 3D fix."""
        return self.fix_type == FixType.FIX_3D
    
    @property
    def is_held(self) -> bool:
        """Check if this is a held position from filter."""
        return self.fix_type == FixType.HOLD
    
    @property
    def position_2d(self) -> Tuple[float, float]:
        """Get 2D position (E, N) in meters."""
        return (self.pos_enu[0], self.pos_enu[1])
    
    @property
    def altitude_m(self) -> float:
        """Get altitude (U) in meters."""
        return self.pos_enu[2]
    
    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            'tag_id': self.tag_id,
            't_solve': self.t_solve,
            'fix_type': self.fix_type.name,
            'pos_enu': self.pos_enu,
            'quality_score': self.quality_score,
            'num_anchors_used': self.num_anchors_used,
            'anchor_ids': self.anchor_ids,
            'residual_m': self.residual_m,
            'pos_std_enu': self.pos_std_enu,
            'geometry_score': self.geometry_score,
            'vel_enu': self.vel_enu,
            'innovation_m': self.innovation_m,
        }


def create_no_fix(
    tag_id: str,
    t_solve: float,
    last_pos_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> PositionEstimate:
    """
    Create a NO_FIX position estimate.
    
    Args:
        tag_id: Tag ID
        t_solve: Solve time
        last_pos_enu: Last known position (default: origin)
        
    Returns:
        PositionEstimate with NO_FIX
    """
    return PositionEstimate(
        tag_id=tag_id,
        t_solve=t_solve,
        fix_type=FixType.NO_FIX,
        pos_enu=last_pos_enu,
        quality_score=0.0,
        num_anchors_used=0,
        anchor_ids=[],
        residual_m=0.0,
    )
