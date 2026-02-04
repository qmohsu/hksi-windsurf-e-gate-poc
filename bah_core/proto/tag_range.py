"""
Tag Range Report Message Schema.

Defines messages for UWB ranging from anchors to tags.
Used in P1-C for tag position multilateration.

Reference: Design Doc Section 7 (Tag Localization Engine)
"""

from dataclasses import dataclass
from typing import Optional
from enum import IntEnum


class TagRangeQuality(IntEnum):
    """Quality indicator for tag UWB range measurements."""
    
    EXCELLENT = 0  # LOS, strong signal
    GOOD = 1       # Likely LOS, acceptable signal
    FAIR = 2       # Possible multipath, weaker signal
    POOR = 3       # Likely NLOS or weak signal
    INVALID = 4    # Failed quality checks


@dataclass
class TagRangeReport:
    """
    UWB range measurement from anchor to tag.
    
    Attributes:
        tag_id: ID of the tag (e.g., "T1")
        anchor_id: ID of the anchor (e.g., "A0")
        distance_m: Measured distance in meters
        t_measurement: UWB measurement timestamp
        t_bah_rx: Time when BAH received this report (solve time)
        quality: Quality indicator for this range
        variance_m2: Range measurement variance (m²), if available
        
    Notes:
        - t_range := t_bah_rx (solve time is when BAH received it)
        - Distance should always be positive
        - t_bah_rx >= t_measurement (causality)
    """
    
    tag_id: str
    anchor_id: str
    distance_m: float
    t_measurement: float
    t_bah_rx: float
    quality: TagRangeQuality = TagRangeQuality.GOOD
    variance_m2: Optional[float] = None
    
    def __post_init__(self):
        """Validate range report after initialization."""
        if self.distance_m < 0:
            raise ValueError(f"Distance cannot be negative: {self.distance_m}")
        
        if self.t_bah_rx < self.t_measurement:
            raise ValueError(
                f"BAH rx time {self.t_bah_rx} before measurement time {self.t_measurement}"
            )
    
    @property
    def t_range(self) -> float:
        """
        Solve time (when to compute anchor positions).
        
        By convention: t_range := t_bah_rx
        """
        return self.t_bah_rx
    
    @property
    def age_at_bah(self) -> float:
        """Time delay between measurement and BAH reception (seconds)."""
        return self.t_bah_rx - self.t_measurement
    
    @property
    def is_valid(self) -> bool:
        """Check if range passes basic validity checks."""
        return (
            self.distance_m > 0 and
            self.quality != TagRangeQuality.INVALID and
            self.age_at_bah >= 0
        )
    
    def get_variance(self) -> float:
        """
        Get range variance, using default if not provided.
        
        Returns:
            Range variance in m²
            
        Notes:
            Default variance based on quality:
            - EXCELLENT: 0.05m (5cm std dev)
            - GOOD: 0.10m (10cm std dev)
            - FAIR: 0.25m (25cm std dev)
            - POOR: 1.0m (1m std dev)
        """
        if self.variance_m2 is not None:
            return self.variance_m2
        
        # Default variances by quality
        quality_variance = {
            TagRangeQuality.EXCELLENT: 0.05 ** 2,
            TagRangeQuality.GOOD: 0.10 ** 2,
            TagRangeQuality.FAIR: 0.25 ** 2,
            TagRangeQuality.POOR: 1.0 ** 2,
            TagRangeQuality.INVALID: 10.0 ** 2,
        }
        
        return quality_variance.get(self.quality, 0.25 ** 2)


@dataclass
class TagRangeBatch:
    """
    Batch of tag range reports for a single solve.
    
    For 3-anchor system, a complete batch has 3 ranges from the same tag.
    
    Attributes:
        tag_id: Tag ID for this batch
        t_solve: Representative solve time (typically max(t_bah_rx))
        ranges: List of TagRangeReport for this tag
    """
    
    tag_id: str
    t_solve: float
    ranges: list  # List[TagRangeReport]
    
    def get_range_by_anchor(self, anchor_id: str) -> Optional[TagRangeReport]:
        """
        Get range from a specific anchor.
        
        Args:
            anchor_id: Anchor ID to find
            
        Returns:
            TagRangeReport if found, None otherwise
        """
        for range_report in self.ranges:
            if range_report.anchor_id == anchor_id:
                return range_report
        return None
    
    def get_valid_ranges(self) -> list:
        """Get all valid ranges in this batch."""
        return [r for r in self.ranges if r.is_valid]
    
    @property
    def num_valid(self) -> int:
        """Number of valid ranges in batch."""
        return len(self.get_valid_ranges())
    
    @property
    def is_complete_for_3_anchors(self) -> bool:
        """Check if batch has all 3 ranges for 3-anchor system."""
        return self.num_valid >= 3
    
    @property
    def anchor_ids(self) -> list:
        """Get list of anchor IDs in this batch."""
        return [r.anchor_id for r in self.ranges]
