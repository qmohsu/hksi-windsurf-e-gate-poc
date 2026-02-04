"""
Protocol Module: Message schemas and versioning.

Implements Rule 70 (network protocols) requirements:
- Schema versioning
- Source IDs, sequence numbers
- Monotonic timestamps
- Defensive parsing
"""

# P1-B imports
from .inter_anchor_range import (
    InterAnchorRangeReport,
    InterAnchorRangeBatch,
    RangeQuality,
)

# P1-C imports
from .tag_range import (
    TagRangeReport,
    TagRangeBatch,
    TagRangeQuality,
)
from .position_estimate import (
    PositionEstimate,
    FixType,
    create_no_fix,
)

# P1-E imports
from .gate_metrics import (
    GateMetrics,
    CrossingEvent,
    create_no_gate_metrics,
)

__all__ = [
    # P1-B
    'InterAnchorRangeReport',
    'InterAnchorRangeBatch',
    'RangeQuality',
    # P1-C
    'TagRangeReport',
    'TagRangeBatch',
    'TagRangeQuality',
    'PositionEstimate',
    'FixType',
    'create_no_fix',
    # P1-E
    'GateMetrics',
    'CrossingEvent',
    'create_no_gate_metrics',
]
