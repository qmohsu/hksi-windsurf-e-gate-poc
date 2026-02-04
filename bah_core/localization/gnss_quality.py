"""
GNSS quality assessment and measurement noise estimation.

Converts GNSS quality indicators (fix type, HDOP) into measurement noise
covariance for Kalman filter updates.

Reference: Design Doc Section 6.3 (AnchorTracker Interface)
"""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple
import numpy as np


class GNSSFixType(Enum):
    """GNSS fix type enumeration."""
    
    NO_FIX = 0
    GPS_FIX = 1          # Standard GPS fix
    DGPS_FIX = 2         # Differential GPS
    PPS_FIX = 3          # PPS (Precision Positioning Service)
    RTK_FIX = 4          # Real-Time Kinematic
    RTK_FLOAT = 5        # RTK Float
    ESTIMATED = 6        # Dead reckoning
    MANUAL = 7           # Manual input
    SIMULATION = 8       # Simulation mode


@dataclass
class GNSSQuality:
    """
    GNSS quality metrics.
    
    Attributes:
        fix_type: Type of GNSS fix
        hdop: Horizontal Dilution of Precision (1.0 = excellent, >5 = poor)
        vdop: Vertical Dilution of Precision (optional)
        num_satellites: Number of satellites used
        age_seconds: Age of differential corrections (for DGPS)
    """
    
    fix_type: GNSSFixType
    hdop: float
    vdop: float = 999.0  # Default to poor if not provided
    num_satellites: int = 0
    age_seconds: float = 0.0
    
    def is_valid(self) -> bool:
        """
        Check if GNSS quality is acceptable for use.
        
        Returns:
            True if fix type is valid and HDOP is reasonable
        """
        if self.fix_type == GNSSFixType.NO_FIX:
            return False
        
        # HDOP threshold: < 10 is acceptable
        if self.hdop > 10.0:
            return False
        
        # Require at least 4 satellites for 3D fix
        if self.num_satellites > 0 and self.num_satellites < 4:
            return False
        
        return True
    
    def is_excellent(self) -> bool:
        """True if GNSS quality is excellent (HDOP < 2, RTK, or many satellites)."""
        if self.fix_type in (GNSSFixType.RTK_FIX, GNSSFixType.DGPS_FIX):
            return True
        
        if self.hdop < 2.0 and self.num_satellites >= 8:
            return True
        
        return False
    
    def is_degraded(self) -> bool:
        """True if GNSS quality is degraded (HDOP > 5 or few satellites)."""
        if self.hdop > 5.0:
            return True
        
        if self.num_satellites > 0 and self.num_satellites < 6:
            return True
        
        return False


def quality_to_noise_std(quality: GNSSQuality) -> Tuple[float, float, float]:
    """
    Convert GNSS quality to measurement noise standard deviations.
    
    Args:
        quality: GNSS quality metrics
        
    Returns:
        Tuple of (sigma_e, sigma_n, sigma_u) in meters
        
    Reference:
    - HDOP=1 (excellent): \u03c3=1m horizontal, 1.5m vertical
    - HDOP=2 (good): \u03c3=2m horizontal, 3m vertical
    - HDOP=5 (fair): \u03c3=5m horizontal, 7.5m vertical
    - HDOP>5 (poor): \u03c3=10m horizontal, 15m vertical
    
    Vertical uncertainty is typically 1.5x horizontal.
    """
    
    # Base noise from fix type
    if quality.fix_type == GNSSFixType.RTK_FIX:
        base_horizontal = 0.02  # 2cm for RTK fixed
        base_vertical = 0.03
    elif quality.fix_type == GNSSFixType.RTK_FLOAT:
        base_horizontal = 0.5   # 50cm for RTK float
        base_vertical = 0.75
    elif quality.fix_type == GNSSFixType.DGPS_FIX:
        base_horizontal = 0.5   # 50cm for DGPS
        base_vertical = 0.75
    else:
        base_horizontal = 1.0   # 1m for standard GPS
        base_vertical = 1.5
    
    # Scale by HDOP
    hdop = max(1.0, min(quality.hdop, 20.0))  # Clamp to [1, 20]
    
    sigma_e = base_horizontal * hdop
    sigma_n = base_horizontal * hdop
    sigma_u = base_vertical * hdop
    
    return (sigma_e, sigma_n, sigma_u)


def quality_to_measurement_covariance(quality: GNSSQuality) -> np.ndarray:
    """
    Convert GNSS quality to measurement noise covariance matrix R.
    
    Args:
        quality: GNSS quality metrics
        
    Returns:
        3x3 measurement covariance matrix R (diagonal)
    """
    sigma_e, sigma_n, sigma_u = quality_to_noise_std(quality)
    
    # Diagonal covariance (assume independent errors in E, N, U)
    R = np.diag([
        sigma_e ** 2,
        sigma_n ** 2,
        sigma_u ** 2,
    ])
    
    return R


def hdop_from_quality_string(quality_str: str) -> float:
    """
    Parse HDOP from common GNSS quality string formats.
    
    Args:
        quality_str: Quality string (e.g., "HDOP=1.2", "1.5", "good")
        
    Returns:
        HDOP value (default 5.0 if unparseable)
    """
    quality_str = quality_str.lower().strip()
    
    # Try to extract number
    import re
    match = re.search(r'(\d+\.?\d*)', quality_str)
    if match:
        return float(match.group(1))
    
    # Qualitative mappings
    if 'excellent' in quality_str or 'rtk' in quality_str:
        return 1.0
    elif 'good' in quality_str or 'dgps' in quality_str:
        return 2.0
    elif 'fair' in quality_str or 'moderate' in quality_str:
        return 4.0
    elif 'poor' in quality_str:
        return 8.0
    else:
        return 5.0  # Default: fair
