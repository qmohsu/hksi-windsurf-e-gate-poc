"""
Inter-Anchor Range Report Message Schema.

Defines messages for UWB ranging between anchors (anchor-to-anchor).
Used in P1-B for anchor network geometry refinement.

Reference: Design Doc Section 6.5 (Anchor Network Fusion)
"""

from dataclasses import dataclass
from typing import Optional
from enum import IntEnum


class RangeQuality(IntEnum):
    """Quality indicator for UWB range measurements."""
    
    EXCELLENT = 0  # LOS, strong signal
    GOOD = 1       # Likely LOS, acceptable signal
    FAIR = 2       # Possible multipath, weaker signal
    POOR = 3       # Likely NLOS or weak signal
    INVALID = 4    # Failed quality checks


@dataclass
class InterAnchorRangeReport:
    """
    UWB range measurement between two anchors.
    
    Attributes:
        anchor_id_i: ID of first anchor (e.g., "A0")
        anchor_id_j: ID of second anchor (e.g., "A1")
        distance_m: Measured distance in meters
        t_measurement: Timestamp of range measurement (monotonic clock)
        t_received: Timestamp when BAH received this report
        quality: Quality indicator for this range
        variance_m2: Range measurement variance (m²), if available
        
    Notes:
        - Distance should always be positive
        - Pair order doesn't matter: (A0, A1) == (A1, A0)
        - t_measurement is the UWB timestamp
        - t_received is when BAH processed it (for staleness checking)
    """
    
    anchor_id_i: str
    anchor_id_j: str
    distance_m: float
    t_measurement: float
    t_received: float
    quality: RangeQuality = RangeQuality.GOOD
    variance_m2: Optional[float] = None
    
    def __post_init__(self):
        """Validate range report after initialization."""
        if self.distance_m < 0:
            raise ValueError(f"Distance cannot be negative: {self.distance_m}")
        
        if self.anchor_id_i == self.anchor_id_j:
            raise ValueError(f"Cannot have range to self: {self.anchor_id_i}")
        
        if self.t_received < self.t_measurement:
            raise ValueError(
                f"Received time {self.t_received} before measurement time {self.t_measurement}"
            )
    
    def get_ordered_pair(self) -> tuple:
        """
        Return anchor IDs in canonical order for consistent key lookup.
        
        Returns:
            Tuple of (id_lower, id_higher) in lexicographic order
            
        Example:
            >>> report = InterAnchorRangeReport("A1", "A0", ...)
            >>> report.get_ordered_pair()
            ('A0', 'A1')
        """
        if self.anchor_id_i < self.anchor_id_j:
            return (self.anchor_id_i, self.anchor_id_j)
        else:
            return (self.anchor_id_j, self.anchor_id_i)
    
    @property
    def age_at_processing(self) -> float:
        """Time delay between measurement and processing (seconds)."""
        return self.t_received - self.t_measurement
    
    @property
    def is_valid(self) -> bool:
        """Check if range passes basic validity checks."""
        return (
            self.distance_m > 0 and
            self.quality != RangeQuality.INVALID and
            self.age_at_processing >= 0
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
            RangeQuality.EXCELLENT: 0.05 ** 2,
            RangeQuality.GOOD: 0.10 ** 2,
            RangeQuality.FAIR: 0.25 ** 2,
            RangeQuality.POOR: 1.0 ** 2,
            RangeQuality.INVALID: 10.0 ** 2,
        }
        
        return quality_variance.get(self.quality, 0.25 ** 2)


@dataclass
class InterAnchorRangeBatch:
    """
    Batch of inter-anchor range reports at approximately the same time.
    
    Useful for collecting ranges for all pairs before running fusion.
    
    Attributes:
        t_batch: Representative timestamp for this batch
        ranges: List of InterAnchorRangeReport
        
    Example:
        For 3 anchors (A0, A1, A2), a complete batch has 3 ranges:
        - A0 ↔ A1
        - A0 ↔ A2
        - A1 ↔ A2
    """
    
    t_batch: float
    ranges: list  # List[InterAnchorRangeReport]
    
    def get_range_by_pair(self, anchor_i: str, anchor_j: str) -> Optional[InterAnchorRangeReport]:
        """
        Get range for a specific anchor pair.
        
        Args:
            anchor_i: First anchor ID
            anchor_j: Second anchor ID
            
        Returns:
            InterAnchorRangeReport if found, None otherwise
        """
        # Create ordered pair for lookup
        if anchor_i < anchor_j:
            target_pair = (anchor_i, anchor_j)
        else:
            target_pair = (anchor_j, anchor_i)
        
        for range_report in self.ranges:
            if range_report.get_ordered_pair() == target_pair:
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
