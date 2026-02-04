"""
Range Gating for Inter-Anchor UWB Measurements.

Implements sanity checks and temporal consistency validation for
inter-anchor range reports before fusion.

Reference: Design Doc Section 6.5.4 (Gating and weighting rules)
"""

from typing import Optional, Dict, Tuple
from dataclasses import dataclass
import time

from bah_core.proto.inter_anchor_range import InterAnchorRangeReport, RangeQuality
from bah_core.metrics import get_metrics


@dataclass
class RangeGatingConfig:
    """
    Configuration for inter-anchor range gating.
    
    Attributes:
        d_min_m: Minimum physically plausible distance (m)
        d_max_m: Maximum operational distance (m)
        v_max_m_s: Maximum relative velocity between anchors (m/s)
        max_age_s: Maximum age for range measurement (s)
        temporal_margin_m: Extra margin for temporal consistency (m)
        min_quality: Minimum acceptable quality level
    """
    
    d_min_m: float = 1.0          # Anchors can't be closer than 1m
    d_max_m: float = 100.0        # Max operational range 100m
    v_max_m_s: float = 2.0        # Buoy drift velocity ~2m/s max
    max_age_s: float = 1.0        # Ranges older than 1s are stale
    temporal_margin_m: float = 0.5  # Extra tolerance for motion
    min_quality: RangeQuality = RangeQuality.FAIR
    
    def __post_init__(self):
        """Validate configuration."""
        assert self.d_min_m > 0, "d_min must be positive"
        assert self.d_max_m > self.d_min_m, "d_max must be greater than d_min"
        assert self.v_max_m_s > 0, "v_max must be positive"
        assert self.max_age_s > 0, "max_age must be positive"


class InterAnchorRangeGate:
    """
    Gate inter-anchor range measurements for outliers.
    
    Applies three types of checks:
    1. Sanity bounds: d_min < d < d_max
    2. Quality threshold: quality >= min_quality
    3. Temporal consistency: |d(t) - d(t-Δt)| < v_max * Δt + margin
    
    Usage:
        gate = InterAnchorRangeGate(config)
        
        if gate.check_range(range_report):
            # Range is valid, use it
            ...
        else:
            # Range failed gating, ignore
            reason = gate.get_rejection_reason(range_report)
            print(f"Rejected: {reason}")
    """
    
    def __init__(self, config: Optional[RangeGatingConfig] = None):
        """
        Initialize range gate.
        
        Args:
            config: Gating configuration (uses defaults if None)
        """
        self.config = config or RangeGatingConfig()
        self.metrics = get_metrics()
        
        # History of accepted ranges for temporal consistency
        # Key: (anchor_i, anchor_j) ordered tuple
        # Value: (distance_m, t_measurement)
        self._range_history: Dict[Tuple[str, str], Tuple[float, float]] = {}
        
        # Track last rejection reason per pair (for debugging)
        self._last_rejection: Dict[Tuple[str, str], str] = {}
    
    def check_range(self, range_report: InterAnchorRangeReport) -> bool:
        """
        Check if range passes all gating criteria.
        
        Args:
            range_report: Inter-anchor range to validate
            
        Returns:
            True if range passes all checks, False otherwise
            
        Side Effects:
            - Updates metrics counters
            - Updates range history if accepted
            - Updates rejection reason if rejected
        """
        pair = range_report.get_ordered_pair()
        distance = range_report.distance_m
        t_meas = range_report.t_measurement
        
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
        if range_report.age_at_processing > self.config.max_age_s:
            self._reject(pair, "stale")
            return False
        
        # Check 5: Temporal consistency (if we have history)
        if pair in self._range_history:
            prev_distance, prev_time = self._range_history[pair]
            dt = t_meas - prev_time
            
            if dt > 0:  # Only check if not duplicate/out-of-order
                expected_max_change = self.config.v_max_m_s * dt + self.config.temporal_margin_m
                actual_change = abs(distance - prev_distance)
                
                if actual_change > expected_max_change:
                    self._reject(pair, "temporal_inconsistent")
                    return False
        
        # All checks passed - accept range
        self._accept(pair, distance, t_meas)
        return True
    
    def get_rejection_reason(self, range_report: InterAnchorRangeReport) -> Optional[str]:
        """
        Get reason why a range was last rejected.
        
        Args:
            range_report: Range to check
            
        Returns:
            Rejection reason string, or None if not rejected recently
        """
        pair = range_report.get_ordered_pair()
        return self._last_rejection.get(pair)
    
    def _accept(self, pair: Tuple[str, str], distance: float, t_meas: float):
        """Record accepted range and update metrics."""
        self._range_history[pair] = (distance, t_meas)
        self.metrics.increment('inter_anchor_ranges_accepted')
        
        # Clear rejection history
        if pair in self._last_rejection:
            del self._last_rejection[pair]
    
    def _reject(self, pair: Tuple[str, str], reason: str):
        """Record rejected range and update metrics."""
        self._last_rejection[pair] = reason
        self.metrics.increment('inter_anchor_ranges_rejected')
        self.metrics.increment_drop(f'inter_anchor_{reason}')
    
    def get_last_accepted_distance(
        self, 
        anchor_i: str, 
        anchor_j: str
    ) -> Optional[Tuple[float, float]]:
        """
        Get last accepted range for anchor pair.
        
        Args:
            anchor_i: First anchor ID
            anchor_j: Second anchor ID
            
        Returns:
            Tuple of (distance_m, t_measurement) if available, None otherwise
        """
        if anchor_i < anchor_j:
            pair = (anchor_i, anchor_j)
        else:
            pair = (anchor_j, anchor_i)
        
        return self._range_history.get(pair)
    
    def reset_history(self):
        """Clear all range history (use when anchors reconfigured)."""
        self._range_history.clear()
        self._last_rejection.clear()
        self.metrics.increment('inter_anchor_gate_reset')
    
    def get_statistics(self) -> dict:
        """Get gating statistics for diagnostics."""
        return {
            'num_pairs_tracked': len(self._range_history),
            'accepted_total': self.metrics.get_counter('inter_anchor_ranges_accepted'),
            'rejected_total': self.metrics.get_counter('inter_anchor_ranges_rejected'),
        }


def create_default_gate() -> InterAnchorRangeGate:
    """
    Create range gate with default configuration for open-water operation.
    
    Returns:
        Configured InterAnchorRangeGate instance
    """
    config = RangeGatingConfig(
        d_min_m=5.0,      # Anchors at least 5m apart (operational)
        d_max_m=50.0,     # Max 50m triangle (Phase 1 scale)
        v_max_m_s=2.0,    # Buoy drift ~2m/s
        max_age_s=1.0,    # Accept only fresh ranges
        temporal_margin_m=0.5,  # 50cm tolerance
        min_quality=RangeQuality.FAIR,
    )
    
    return InterAnchorRangeGate(config)
