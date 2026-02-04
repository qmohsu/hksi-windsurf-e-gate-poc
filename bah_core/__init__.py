"""
Buoy Anchor Hub (BAH) Core Package.

Phase 1: 3-Anchor UWB + GNSS positioning system for windsurf start-line monitoring.

Package structure:
- io: Network I/O, message parsing, bounded queues
- proto: Message schemas and protocol definitions
- localization: Anchor tracking, tag estimation, coordinate transforms
- domain: Business logic (gate metrics, crossing detection)
- metrics: Diagnostics, counters, histograms

Design Doc: docs/design_doc.md
Dev Plan: docs/dev_plan.md
"""

__version__ = "0.1.0-phase1"
__author__ = "HKSI UWB Team"

# Convenience imports (add as modules are implemented)
# from .metrics import get_metrics
# from .localization import AnchorTracker, CoordinateConverter
