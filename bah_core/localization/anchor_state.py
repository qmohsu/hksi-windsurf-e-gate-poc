"""
Anchor state representation.

Output of AnchorTracker.predict() - provides anchor position, velocity,
uncertainty, and quality flags at a specific time.

Reference: Design Doc Section 6 (Anchor Location Estimation)
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np

from .gnss_quality import GNSSQuality


@dataclass
class AnchorState:
    """
    Anchor state at a specific time.
    
    This is the output of AnchorTracker.predict(t_range) and provides
    everything needed to use this anchor in tag localization.
    
    Attributes:
        anchor_id: Anchor identifier (e.g., "A0", "A1")
        time: Time of this state (seconds, BAH monotonic clock)
        pos_enu: Position in ENU frame (E, N, U) meters
        vel_enu: Velocity in ENU frame (vE, vN, vU) m/s (optional)
        pos_cov: Position covariance 3x3 (uncertainty in meters^2)
        last_gnss_time: Time of last GNSS update
        last_gnss_quality: Quality of last GNSS fix
        is_valid: True if this state is usable
        is_degraded: True if quality is poor (stale, high uncertainty)
    """
    
    anchor_id: str
    time: float
    pos_enu: Tuple[float, float, float]
    vel_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    pos_cov: np.ndarray = None  # 3x3 covariance matrix
    last_gnss_time: float = 0.0
    last_gnss_quality: GNSSQuality = None
    is_valid: bool = True
    is_degraded: bool = False
    
    def __post_init__(self):
        """Validate and set defaults."""
        if self.pos_cov is None:
            # Default: 1m std in each direction
            self.pos_cov = np.eye(3) * 1.0
    
    @property
    def last_fix_age(self) -> float:
        """
        Age of last GNSS fix in seconds.
        
        Returns:
            Time since last GNSS update
        """
        return self.time - self.last_gnss_time
    
    @property
    def position_std(self) -> Tuple[float, float, float]:
        """
        Position standard deviations (sqrt of diagonal covariance).
        
        Returns:
            (std_e, std_n, std_u) in meters
        """
        return (
            np.sqrt(self.pos_cov[0, 0]),
            np.sqrt(self.pos_cov[1, 1]),
            np.sqrt(self.pos_cov[2, 2]),
        )
    
    @property
    def position_uncertainty_m(self) -> float:
        """
        Overall position uncertainty (2D RMS).
        
        Returns:
            RMS uncertainty in meters (horizontal only)
        """
        std_e, std_n, _ = self.position_std
        return np.sqrt(std_e**2 + std_n**2)
    
    def to_dict(self) -> dict:
        """
        Serialize to dictionary.
        
        Returns:
            Dict representation suitable for JSON/logging
        """
        return {
            'anchor_id': self.anchor_id,
            'time': self.time,
            'pos_enu': {
                'e': self.pos_enu[0],
                'n': self.pos_enu[1],
                'u': self.pos_enu[2],
            },
            'vel_enu': {
                'vE': self.vel_enu[0],
                'vN': self.vel_enu[1],
                'vU': self.vel_enu[2],
            },
            'position_uncertainty_m': self.position_uncertainty_m,
            'last_fix_age': self.last_fix_age,
            'is_valid': self.is_valid,
            'is_degraded': self.is_degraded,
            'last_gnss_quality': {
                'fix_type': self.last_gnss_quality.fix_type.name if self.last_gnss_quality else None,
                'hdop': self.last_gnss_quality.hdop if self.last_gnss_quality else None,
            } if self.last_gnss_quality else None,
        }


@dataclass
class AnchorStateTable:
    """
    Table of anchor states at a specific time.
    
    Provides convenient access to multiple anchor states for tag solving.
    """
    
    time: float
    anchors: dict  # anchor_id -> AnchorState
    
    def get_valid_anchors(self) -> list:
        """
        Get list of valid anchor IDs.
        
        Returns:
            List of anchor IDs with is_valid=True
        """
        return [
            anchor_id for anchor_id, state in self.anchors.items()
            if state.is_valid
        ]
    
    def has_minimum_anchors(self, min_count: int = 3) -> bool:
        """
        Check if we have minimum number of valid anchors.
        
        Args:
            min_count: Minimum required anchors (default 3)
            
        Returns:
            True if we have at least min_count valid anchors
        """
        return len(self.get_valid_anchors()) >= min_count
    
    def get_positions_for_solve(self) -> list:
        """
        Get anchor positions in format for tag solver.
        
        Returns:
            List of (anchor_id, pos_enu) tuples for valid anchors
        """
        result = []
        for anchor_id in sorted(self.get_valid_anchors()):
            state = self.anchors[anchor_id]
            result.append((anchor_id, state.pos_enu))
        return result
    
    def get_total_uncertainty(self) -> float:
        """
        Get combined uncertainty across all valid anchors.
        
        Returns:
            RMS uncertainty in meters
        """
        valid_anchors = self.get_valid_anchors()
        if not valid_anchors:
            return float('inf')
        
        uncertainties = [
            self.anchors[aid].position_uncertainty_m
            for aid in valid_anchors
        ]
        
        # RMS of individual uncertainties
        return np.sqrt(np.mean([u**2 for u in uncertainties]))
