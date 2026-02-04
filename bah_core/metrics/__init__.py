"""
Metrics Module: Diagnostics, counters, histograms.

Implements Rule 80 (metrics-logging) requirements:
- Counters: packets_in, packets_dropped_by_reason, parse_errors, etc.
- Histograms: latency, residuals, prediction_horizon
- Drop reason codes per Rule 05 (no silent failures)

Usage:
    from bah_core.metrics import get_metrics
    
    metrics = get_metrics()
    metrics.increment('packets_in')
    metrics.increment_drop('outlier')
    metrics.record_histogram('latency_ms', 1.23)
"""

from .counters import MetricsCollector

# Global singleton for easy access
_global_metrics = None


def get_metrics() -> MetricsCollector:
    """
    Get the global metrics collector singleton.
    
    Returns:
        MetricsCollector instance
    """
    global _global_metrics
    if _global_metrics is None:
        _global_metrics = MetricsCollector()
    return _global_metrics


def reset_metrics():
    """Reset global metrics (for testing)."""
    global _global_metrics
    _global_metrics = MetricsCollector()


__all__ = ['MetricsCollector', 'get_metrics', 'reset_metrics']
