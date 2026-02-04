"""
Localization Module: Anchor tracking, tag estimation, coordinate transforms.

Implements Design Doc sections:
- Section 5: Coordinate frames (ENU, plane)
- Section 6: Anchor location estimation (GNSS + UWB fusion)
- Section 7: Tag localization engine

Key classes:
- CoordinateConverter: GNSS <-> ENU <-> Plane transforms
- AnchorTracker: Per-anchor Kalman filter for continuous GNSS state
- AnchorNetworkFusion: Inter-anchor UWB ranging fusion (P1-B)
- TagSolver: Time-aligned 3-anchor multilateration
- RangeGating: Outlier rejection for UWB ranges
- TagKinematicFilter: Smoothing and holdover
"""

# Core imports (P1-A)
from .coordinate_converter import (
    CoordinateConverter,
    GNSSCoordinate,
    PlaneCoordinate,
    create_virtual_anchor_coordinates,
)
from .anchor_tracker import AnchorTracker
from .anchor_state import AnchorState, AnchorStateTable
from .gnss_quality import (
    GNSSQuality,
    GNSSFixType,
    quality_to_noise_std,
    quality_to_measurement_covariance,
)

# P1-B imports (Inter-Anchor Fusion)
from .range_gating import (
    InterAnchorRangeGate,
    RangeGatingConfig,
    create_default_gate,
)
from .anchor_network_fusion import (
    AnchorNetworkFusion,
    FusionConfig,
    FusionResult,
)

# P1-C imports (Tag Multilateration)
from .tag_solver import (
    TagSolver,
    TagSolverConfig,
)

# P1-D imports (Robustness)
from .tag_range_gating import (
    TagRangeGate,
    TagRangeGatingConfig,
    create_default_tag_gate,
)
from .tag_kinematic_filter import (
    TagKinematicFilter,
    TagKinematicFilterConfig,
)
from .plausibility_checker import (
    PlausibilityChecker,
    PlausibilityConfig,
    create_default_checker,
)
from .robust_tag_positioning import (
    RobustTagPositioningPipeline,
    RobustPositioningConfig,
    create_default_pipeline,
)

__all__ = [
    # Coordinate conversion
    'CoordinateConverter',
    'GNSSCoordinate',
    'PlaneCoordinate',
    'create_virtual_anchor_coordinates',
    # Anchor tracking (P1-A)
    'AnchorTracker',
    'AnchorState',
    'AnchorStateTable',
    'GNSSQuality',
    'GNSSFixType',
    'quality_to_noise_std',
    'quality_to_measurement_covariance',
    # Inter-anchor fusion (P1-B)
    'InterAnchorRangeGate',
    'RangeGatingConfig',
    'create_default_gate',
    'AnchorNetworkFusion',
    'FusionConfig',
    'FusionResult',
    # Tag multilateration (P1-C)
    'TagSolver',
    'TagSolverConfig',
    # Robustness (P1-D)
    'TagRangeGate',
    'TagRangeGatingConfig',
    'create_default_tag_gate',
    'TagKinematicFilter',
    'TagKinematicFilterConfig',
    'PlausibilityChecker',
    'PlausibilityConfig',
    'create_default_checker',
    'RobustTagPositioningPipeline',
    'RobustPositioningConfig',
    'create_default_pipeline',
]
