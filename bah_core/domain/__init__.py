"""
Domain Module: Race-specific logic and gate metrics.

Implements:
- Gate geometry calculation
- Crossing detection
- Race timing logic
"""

# P1-E imports
from .gate_calculator import (
    GateCalculator,
    GateConfig,
    create_default_gate_calculator,
)

__all__ = [
    # P1-E
    'GateCalculator',
    'GateConfig',
    'create_default_gate_calculator',
]
