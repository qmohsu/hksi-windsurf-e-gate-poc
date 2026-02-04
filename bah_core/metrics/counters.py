"""
Metrics counters and histograms implementation.

Provides thread-safe counters for:
- Packet counts (in, dropped, parsed)
- Drop reasons (parse_error, stale, outlier, etc.)
- Solver statistics (successes, failures)
- Timing histograms (latency, prediction horizon)

Per Rule 80: Every dropped measurement must log a reason code.
Per Rule 05: No silent failures.
"""

import threading
import time
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from collections import defaultdict
import statistics


@dataclass
class CounterSnapshot:
    """Snapshot of counter state at a point in time."""
    
    timestamp: float
    counters: Dict[str, int]
    drop_reasons: Dict[str, int]
    histograms: Dict[str, List[float]]
    
    def total_dropped(self) -> int:
        """Total packets dropped across all reasons."""
        return sum(self.drop_reasons.values())
    
    def drop_rate(self, total_packets: int) -> float:
        """Calculate drop rate as percentage."""
        if total_packets == 0:
            return 0.0
        return (self.total_dropped() / total_packets) * 100.0


class MetricsCollector:
    """
    Thread-safe metrics collection.
    
    Implements Rule 80 (metrics-logging) requirements for diagnostic counters.
    
    Usage:
        collector = MetricsCollector()
        collector.increment('packets_in')
        collector.increment_drop('outlier')
        collector.record_histogram('latency_ms', 1.23)
        
        snapshot = collector.snapshot()
        print(f"Total dropped: {snapshot.total_dropped()}")
    """
    
    # Standard drop reason codes (per Rule 05)
    DROP_REASONS = {
        'parse_error': 'Malformed message, failed to parse',
        'stale': 'Timestamp too old',
        'out_of_order': 'Sequence number regression',
        'outlier': 'Failed range gating',
        'queue_full': 'Bounded queue overflow',
        'insufficient_anchors': 'Less than 3 valid anchors',
        'solver_failed': 'Trilateration/KF failed',
        'gnss_poor_quality': 'GNSS quality below threshold',
        'nlos_suspected': 'NLOS/multipath suspected',
    }
    
    def __init__(self):
        """Initialize metrics collector."""
        self._lock = threading.Lock()
        self._counters: Dict[str, int] = defaultdict(int)
        self._drop_reasons: Dict[str, int] = defaultdict(int)
        self._histograms: Dict[str, List[float]] = defaultdict(list)
        self._start_time = time.time()
        
        # Initialize standard counters to 0 for consistent reporting
        self._init_standard_counters()
    
    def _init_standard_counters(self):
        """Initialize standard counter keys."""
        standard_counters = [
            'packets_in',
            'packets_processed',
            'parse_errors',
            'outlier_rejections',
            'solver_failures',
            'gnss_updates',
            'stale_fixes',
            'position_estimates',
        ]
        
        with self._lock:
            for counter in standard_counters:
                if counter not in self._counters:
                    self._counters[counter] = 0
            
            # Initialize all drop reasons to 0
            for reason in self.DROP_REASONS:
                if reason not in self._drop_reasons:
                    self._drop_reasons[reason] = 0
    
    def increment(self, counter_name: str, value: int = 1):
        """
        Increment a counter by value.
        
        Args:
            counter_name: Name of counter to increment
            value: Amount to increment (default 1)
        """
        with self._lock:
            self._counters[counter_name] += value
    
    def increment_drop(self, reason: str, value: int = 1):
        """
        Increment drop counter for specific reason.
        
        Args:
            reason: Drop reason code (should be in DROP_REASONS)
            value: Amount to increment (default 1)
        """
        if reason not in self.DROP_REASONS:
            # Log unknown reason but still count it
            print(f"Warning: Unknown drop reason '{reason}'")
        
        with self._lock:
            self._drop_reasons[reason] += value
            # Also increment general counter
            self._counters['packets_dropped'] += value
    
    def get_counter(self, counter_name: str) -> int:
        """
        Get current value of a counter.
        
        Args:
            counter_name: Name of counter
            
        Returns:
            Current counter value
        """
        with self._lock:
            return self._counters.get(counter_name, 0)
    
    def record_histogram(self, histogram_name: str, value: float, max_samples: int = 10000):
        """
        Record a value in a histogram.
        
        Args:
            histogram_name: Name of histogram
            value: Value to record
            max_samples: Maximum samples to keep (prevents unbounded growth)
        """
        with self._lock:
            samples = self._histograms[histogram_name]
            samples.append(value)
            
            # Keep only recent samples to bound memory
            if len(samples) > max_samples:
                # Keep most recent half
                self._histograms[histogram_name] = samples[-max_samples//2:]
    
    def get_histogram_stats(self, histogram_name: str) -> Optional[Dict[str, float]]:
        """
        Get statistics for a histogram.
        
        Args:
            histogram_name: Name of histogram
            
        Returns:
            Dict with min, max, mean, median, p95, p99, count
            None if histogram is empty
        """
        with self._lock:
            samples = self._histograms.get(histogram_name, [])
            
            if not samples:
                return None
            
            sorted_samples = sorted(samples)
            count = len(sorted_samples)
            
            return {
                'count': count,
                'min': sorted_samples[0],
                'max': sorted_samples[-1],
                'mean': statistics.mean(sorted_samples),
                'median': statistics.median(sorted_samples),
                'p95': sorted_samples[int(count * 0.95)] if count > 1 else sorted_samples[0],
                'p99': sorted_samples[int(count * 0.99)] if count > 1 else sorted_samples[0],
            }
    
    def snapshot(self) -> CounterSnapshot:
        """
        Get a snapshot of current metrics state.
        
        Returns:
            CounterSnapshot with copies of all metrics
        """
        with self._lock:
            return CounterSnapshot(
                timestamp=time.time(),
                counters=dict(self._counters),
                drop_reasons=dict(self._drop_reasons),
                histograms={k: list(v) for k, v in self._histograms.items()},
            )
    
    def reset(self):
        """Reset all metrics (useful for testing)."""
        with self._lock:
            self._counters.clear()
            self._drop_reasons.clear()
            self._histograms.clear()
            self._start_time = time.time()
            self._init_standard_counters()
    
    def get_uptime(self) -> float:
        """Get uptime in seconds since initialization."""
        return time.time() - self._start_time
    
    def print_summary(self):
        """Print human-readable metrics summary."""
        snapshot = self.snapshot()
        uptime = self.get_uptime()
        
        print("\n" + "=" * 70)
        print(f"  METRICS SUMMARY (uptime: {uptime:.1f}s)")
        print("=" * 70)
        
        # Counters
        print("\nCOUNTERS:")
        for name, value in sorted(snapshot.counters.items()):
            print(f"  {name:30s}: {value:8d}")
        
        # Drop reasons
        total_dropped = snapshot.total_dropped()
        if total_dropped > 0:
            print("\nDROP REASONS:")
            for reason, count in sorted(snapshot.drop_reasons.items()):
                if count > 0:
                    pct = (count / total_dropped) * 100
                    print(f"  {reason:30s}: {count:8d} ({pct:5.1f}%)")
        
        # Histograms
        if snapshot.histograms:
            print("\nHISTOGRAMS:")
            for name in sorted(snapshot.histograms.keys()):
                stats = self.get_histogram_stats(name)
                if stats:
                    print(f"  {name}:")
                    print(f"    count={stats['count']}, mean={stats['mean']:.3f}, "
                          f"p95={stats['p95']:.3f}, p99={stats['p99']:.3f}")
        
        print("=" * 70 + "\n")
