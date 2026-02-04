"""
Range Gating for Tag UWB Measurements.

Implements sanity checks and validation for tag range reports before
multilateration solving. With only 3 anchors, there's no redundancy,
so gating is critical to prevent "teleporting" from outliers.

Reference: Design Doc Section 7.3 (Robustness and Outlier Handling)
"""

from typing import Optional, Dict, Tuple
from dataclasses import dataclass
import time

from bah_core.proto.tag_range import TagRangeReport, TagRangeQuality
from bah_core.metrics import get_metrics


@dataclass
class TagRangeGatingConfig:
    """
    Configuration for tag range gating.
    
    Attributes:
        d_min_m: Minimum physically plausible distance (m)
        d_max_m: Maximum operational distance (m)
        max_age_s: Maximum age for range measurement (s)
        min_quality: Minimum acceptable quality level
    """
    
    d_min_m: float = 0.5          # Tags can't be closer than 50cm to anchor
    d_max_m: float = 50.0         # Max operational range 50m
    max_age_s: float = 1.0        # Ranges older than 1s are stale
    min_quality: TagRangeQuality = TagRangeQuality.FAIR
    
    def __post_init__(self):
        """Validate configuration."""
        assert self.d_min_m > 0, "d_min must be positive"
        assert self.d_max_m > self.d_min_m, "d_max must be greater than d_min"
        assert self.max_age_s > 0, "max_age must be positive"


class TagRangeGate:
    """
    Gate tag range measurements for outliers.
    
    Applies checks:
    1. Sanity bounds: d_min < d < d_max
    2. Quality threshold: quality >= min_quality
    3. Age threshold: age < max_age
    
    Usage:
        gate = TagRangeGate(config)
        
        if gate.check_range(range_report):
            # Range is valid, use it
            ...
        else:
            # Range failed gating, reject
            reason = gate.get_rejection_reason(range_report)
            print(f"Rejected: {reason}")
    
    Notes:
        - With 3 anchors, no redundancy for outlier removal
        - Gating is the primary defense against bad ranges
        - Temporal consistency not used (tag moves fast)
    """
    
    def __init__(self, config: Optional[TagRangeGatingConfig] = None):
        """
        Initialize range gate.
        
        Args:
            config: Gating configuration (uses defaults if None)
        """
        self.config = config or TagRangeGatingConfig()
        self.metrics = get_metrics()
        
        # Track last rejection reason per tag-anchor pair (for debugging)
        self._last_rejection: Dict[Tuple[str, str], str] = {}
    
    def check_range(self, range_report: TagRangeReport) -> bool:
        """
        Check if range passes all gating criteria.
        
        Args:
            range_report: Tag range to validate
            
        Returns:
            True if range passes all checks, False otherwise
            
        Side Effects:
            - Updates metrics counters
            - Updates rejection reason if rejected
        """
        pair = (range_report.tag_id, range_report.anchor_id)
        distance = range_report.distance_m
        
        # Check 1: Basic validity from message
        if not range_report.is_valid:
            self._reject(pair, "invalid_message")
            return False
        
        # Check 2: Sanity bounds
        if distance < self.config.d_min_m:
            self._reject(pair, "too_close")
            return False
        
        if distance > self.config.d_max_m:
            self._reject(pair, "too_far")
            return False
        
        # Check 3: Quality threshold
        if range_report.quality > self.config.min_quality:
            self._reject(pair, "poor_quality")
            return False
        
        # Check 4: Age threshold
        if range_report.age_at_bah > self.config.max_age_s:
            self._reject(pair, "stale")
            return False
        
        # All checks passed - accept range
        self._accept(pair)
        return True
    
    def check_batch(self, ranges: list) -> list:
        """
        Check a batch of ranges and return only valid ones.
        
        Args:
            ranges: List of TagRangeReport
            
        Returns:
            List of valid TagRangeReport
        """
        return [r for r in ranges if self.check_range(r)]
    
    def get_rejection_reason(self, range_report: TagRangeReport) -> Optional[str]:
        """
        Get reason why a range was last rejected.
        
        Args:
            range_report: Range to check
            
        Returns:
            Rejection reason string, or None if not rejected recently
        """
        pair = (range_report.tag_id, range_report.anchor_id)
        return self._last_rejection.get(pair)
    
    def _accept(self, pair: Tuple[str, str]):
        """Record accepted range and update metrics."""
        self.metrics.increment('tag_ranges_accepted')
        
        # Clear rejection history
        if pair in self._last_rejection:
            del self._last_rejection[pair]
    
    def _reject(self, pair: Tuple[str, str], reason: str):
        """Record rejected range and update metrics."""
        self._last_rejection[pair] = reason
        self.metrics.increment('tag_ranges_rejected')
        self.metrics.increment_drop(f'tag_range_{reason}')
    
    def get_statistics(self) -> dict:
        """Get gating statistics for diagnostics."""
        return {
            'accepted_total': self.metrics.get_counter('tag_ranges_accepted'),
            'rejected_total': self.metrics.get_counter('tag_ranges_rejected'),
        }


def create_default_tag_gate() -> TagRangeGate:
    """
    Create range gate with default configuration for open-water operation.
    
    Returns:
        Configured TagRangeGate instance
    """
    config = TagRangeGatingConfig(
        d_min_m=1.0,      # Tags at least 1m from anchors
        d_max_m=50.0,     # Max 50m range (Phase 1 scale)
        max_age_s=1.0,    # Accept only fresh ranges
        min_quality=TagRangeQuality.FAIR,
    )
    
    return TagRangeGate(config)
