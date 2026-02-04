"""
Unit tests for metrics module.

Tests cover:
- Counter increment (single-threaded and multi-threaded)
- Drop reason tracking
- Histogram recording and statistics
- Snapshot and reset functionality
- Thread safety

Reference: Rule 80 (metrics-logging), Rule 05 (no silent failures)
"""

import threading
import time
from typing import List

import pytest

from bah_core.metrics import MetricsCollector, get_metrics, reset_metrics
from bah_core.metrics.counters import CounterSnapshot


class TestMetricsCollectorBasic:
    """Tests for basic metrics collector functionality."""

    def test_initialization(self):
        """Test that metrics collector initializes correctly."""
        collector = MetricsCollector()
        
        # Standard counters should be initialized to 0
        assert collector.get_counter('packets_in') == 0
        assert collector.get_counter('parse_errors') == 0
        
        # Unknown counter should return 0
        assert collector.get_counter('unknown_counter') == 0

    def test_increment_counter(self):
        """Test incrementing a counter."""
        collector = MetricsCollector()
        
        collector.increment('packets_in')
        assert collector.get_counter('packets_in') == 1
        
        collector.increment('packets_in', 5)
        assert collector.get_counter('packets_in') == 6

    def test_increment_drop_with_valid_reason(self):
        """Test incrementing drop counter with valid reason."""
        collector = MetricsCollector()
        
        collector.increment_drop('outlier')
        assert collector.get_counter('packets_dropped') == 1
        
        snapshot = collector.snapshot()
        assert snapshot.drop_reasons['outlier'] == 1

    def test_increment_drop_unknown_reason(self, capfd):
        """Test incrementing drop counter with unknown reason logs warning."""
        collector = MetricsCollector()
        
        collector.increment_drop('unknown_reason')
        
        captured = capfd.readouterr()
        assert 'unknown_reason' in captured.out.lower()
        
        # Should still be counted
        assert collector.get_counter('packets_dropped') == 1

    def test_multiple_drop_reasons(self):
        """Test tracking multiple drop reasons."""
        collector = MetricsCollector()
        
        collector.increment_drop('parse_error', 3)
        collector.increment_drop('outlier', 5)
        collector.increment_drop('stale', 2)
        
        snapshot = collector.snapshot()
        
        assert snapshot.drop_reasons['parse_error'] == 3
        assert snapshot.drop_reasons['outlier'] == 5
        assert snapshot.drop_reasons['stale'] == 2
        assert snapshot.total_dropped() == 10


class TestHistograms:
    """Tests for histogram functionality."""

    def test_record_histogram(self):
        """Test recording values in histogram."""
        collector = MetricsCollector()
        
        collector.record_histogram('latency_ms', 1.23)
        collector.record_histogram('latency_ms', 2.45)
        collector.record_histogram('latency_ms', 1.80)
        
        stats = collector.get_histogram_stats('latency_ms')
        
        assert stats is not None
        assert stats['count'] == 3
        assert abs(stats['mean'] - 1.826) < 0.01
        assert stats['min'] == 1.23
        assert stats['max'] == 2.45

    def test_histogram_empty(self):
        """Test getting stats for empty histogram."""
        collector = MetricsCollector()
        
        stats = collector.get_histogram_stats('nonexistent')
        assert stats is None

    def test_histogram_percentiles(self):
        """Test histogram percentile calculations."""
        collector = MetricsCollector()
        
        # Record 100 values from 0 to 99
        for i in range(100):
            collector.record_histogram('test', float(i))
        
        stats = collector.get_histogram_stats('test')
        
        assert stats['count'] == 100
        assert stats['min'] == 0.0
        assert stats['max'] == 99.0
        assert 49 < stats['median'] < 51
        assert 94 < stats['p95'] < 96
        assert 98 < stats['p99'] < 100

    def test_histogram_max_samples_bounded(self):
        """Test that histograms are bounded to prevent memory growth."""
        collector = MetricsCollector()
        
        # Record more than max_samples (default 10000)
        for i in range(15000):
            collector.record_histogram('test', float(i), max_samples=1000)
        
        snapshot = collector.snapshot()
        
        # Should be trimmed to half of max_samples
        assert len(snapshot.histograms['test']) <= 500


class TestSnapshot:
    """Tests for snapshot functionality."""

    def test_snapshot_creates_copy(self):
        """Test that snapshot creates independent copy."""
        collector = MetricsCollector()
        
        collector.increment('packets_in', 10)
        snapshot1 = collector.snapshot()
        
        collector.increment('packets_in', 5)
        snapshot2 = collector.snapshot()
        
        # Snapshots should be independent
        assert snapshot1.counters['packets_in'] == 10
        assert snapshot2.counters['packets_in'] == 15

    def test_snapshot_timestamp(self):
        """Test snapshot includes timestamp."""
        collector = MetricsCollector()
        
        before = time.time()
        snapshot = collector.snapshot()
        after = time.time()
        
        assert before <= snapshot.timestamp <= after

    def test_snapshot_drop_rate(self):
        """Test snapshot drop rate calculation."""
        collector = MetricsCollector()
        
        collector.increment('packets_in', 100)
        collector.increment_drop('outlier', 5)
        collector.increment_drop('stale', 3)
        
        snapshot = collector.snapshot()
        
        rate = snapshot.drop_rate(100)
        assert abs(rate - 8.0) < 0.01  # 8% drop rate


class TestReset:
    """Tests for reset functionality."""

    def test_reset_clears_counters(self):
        """Test that reset clears all counters."""
        collector = MetricsCollector()
        
        collector.increment('packets_in', 100)
        collector.increment_drop('outlier', 5)
        collector.record_histogram('latency_ms', 1.23)
        
        collector.reset()
        
        assert collector.get_counter('packets_in') == 0
        assert collector.get_counter('packets_dropped') == 0
        
        snapshot = collector.snapshot()
        assert snapshot.total_dropped() == 0
        assert not snapshot.histograms

    def test_reset_reinitializes_standard_counters(self):
        """Test that reset reinitializes standard counters to 0."""
        collector = MetricsCollector()
        
        collector.increment('packets_in', 100)
        collector.reset()
        
        # Standard counters should exist after reset
        assert 'packets_in' in collector.snapshot().counters


class TestThreadSafety:
    """Tests for thread-safe operations."""

    def test_concurrent_increment(self):
        """Test that concurrent increments are thread-safe."""
        collector = MetricsCollector()
        num_threads = 10
        increments_per_thread = 1000
        
        def worker():
            for _ in range(increments_per_thread):
                collector.increment('packets_in')
        
        threads = [threading.Thread(target=worker) for _ in range(num_threads)]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        expected = num_threads * increments_per_thread
        actual = collector.get_counter('packets_in')
        
        assert actual == expected, f"Expected {expected}, got {actual}"

    def test_concurrent_drop_reasons(self):
        """Test that concurrent drop reason increments are thread-safe."""
        collector = MetricsCollector()
        num_threads = 5
        increments_per_thread = 200
        
        def worker(reason: str):
            for _ in range(increments_per_thread):
                collector.increment_drop(reason)
        
        threads = [
            threading.Thread(target=worker, args=(reason,))
            for reason in ['outlier', 'stale', 'parse_error']
            for _ in range(num_threads)
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        snapshot = collector.snapshot()
        
        # Each reason should have num_threads * increments_per_thread
        expected = num_threads * increments_per_thread
        assert snapshot.drop_reasons['outlier'] == expected
        assert snapshot.drop_reasons['stale'] == expected
        assert snapshot.drop_reasons['parse_error'] == expected

    def test_concurrent_histogram_recording(self):
        """Test that concurrent histogram recording is thread-safe."""
        collector = MetricsCollector()
        num_threads = 10
        samples_per_thread = 100
        
        def worker(value_offset: float):
            for i in range(samples_per_thread):
                collector.record_histogram('test', value_offset + i)
        
        threads = [
            threading.Thread(target=worker, args=(t * 1000,))
            for t in range(num_threads)
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        stats = collector.get_histogram_stats('test')
        
        # Should have all samples
        assert stats['count'] == num_threads * samples_per_thread


class TestGlobalSingleton:
    """Tests for global metrics singleton."""

    def test_get_metrics_returns_same_instance(self):
        """Test that get_metrics() returns the same instance."""
        metrics1 = get_metrics()
        metrics2 = get_metrics()
        
        assert metrics1 is metrics2

    def test_global_metrics_persists_data(self):
        """Test that global metrics persists data across calls."""
        metrics1 = get_metrics()
        metrics1.increment('test_counter', 42)
        
        metrics2 = get_metrics()
        assert metrics2.get_counter('test_counter') == 42

    def test_reset_metrics_creates_new_instance(self):
        """Test that reset_metrics() creates fresh instance."""
        metrics1 = get_metrics()
        metrics1.increment('test_counter', 100)
        
        reset_metrics()
        
        metrics2 = get_metrics()
        assert metrics2.get_counter('test_counter') == 0


class TestDropReasonCodes:
    """Tests for standard drop reason codes."""

    def test_all_standard_drop_reasons_defined(self):
        """Test that all standard drop reasons are defined."""
        collector = MetricsCollector()
        
        expected_reasons = [
            'parse_error',
            'stale',
            'out_of_order',
            'outlier',
            'queue_full',
            'insufficient_anchors',
            'solver_failed',
            'gnss_poor_quality',
            'nlos_suspected',
        ]
        
        for reason in expected_reasons:
            assert reason in collector.DROP_REASONS

    def test_drop_reasons_initialized_to_zero(self):
        """Test that all drop reasons are initialized to 0."""
        collector = MetricsCollector()
        snapshot = collector.snapshot()
        
        for reason in collector.DROP_REASONS:
            assert snapshot.drop_reasons[reason] == 0


class TestUptime:
    """Tests for uptime tracking."""

    def test_uptime_increases(self):
        """Test that uptime increases over time."""
        collector = MetricsCollector()
        
        uptime1 = collector.get_uptime()
        time.sleep(0.1)
        uptime2 = collector.get_uptime()
        
        assert uptime2 > uptime1
        assert (uptime2 - uptime1) >= 0.1


class TestPrintSummary:
    """Tests for print_summary functionality."""

    def test_print_summary_no_crash(self, capsys):
        """Test that print_summary doesn't crash with various data."""
        collector = MetricsCollector()
        
        # Add various metrics
        collector.increment('packets_in', 100)
        collector.increment_drop('outlier', 5)
        collector.record_histogram('latency_ms', 1.23)
        
        # Should not crash
        collector.print_summary()
        
        captured = capsys.readouterr()
        assert 'METRICS SUMMARY' in captured.out
        assert 'packets_in' in captured.out
