"""
DEPRECATED: This module has been moved to bah_core.localization.coordinate_converter

This wrapper is provided for backward compatibility during Phase 1 migration.
Please update your imports to:
    from bah_core.localization import CoordinateConverter, GNSSCoordinate, PlaneCoordinate

This wrapper will be removed in Phase 2.
"""

import warnings
from bah_core.localization.coordinate_converter import (
    CoordinateConverter,
    GNSSCoordinate,
    PlaneCoordinate,
    create_virtual_anchor_coordinates,
)

# Issue deprecation warning
warnings.warn(
    "coordinate_converter module at root is deprecated. "
    "Use 'from bah_core.localization import CoordinateConverter' instead. "
    "Root-level import will be removed in Phase 2.",
    DeprecationWarning,
    stacklevel=2
)

__all__ = [
    'CoordinateConverter',
    'GNSSCoordinate',
    'PlaneCoordinate',
    'create_virtual_anchor_coordinates',
]
