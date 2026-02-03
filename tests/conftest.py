"""
Pytest configuration and shared fixtures for HKSI UWB Positioning System tests.

This module provides reusable fixtures for testing trilateration algorithms,
coordinate conversion, network protocols, and system integration.
"""

import sys
import math
from pathlib import Path
from typing import Dict, List, Tuple

import pytest

# Add project root to path for imports
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from coordinate_converter import (
    CoordinateConverter,
    GNSSCoordinate,
    PlaneCoordinate,
    create_virtual_anchor_coordinates,
)
from trilateration_wrapper import TrilaterationWrapper, PythonTrilateration, Vec3D


# =============================================================================
# Anchor Configuration Fixtures
# =============================================================================


@pytest.fixture
def virtual_gnss_config() -> Dict:
    """
    Standard virtual GNSS configuration for testing.

    Returns an equilateral triangle anchor layout with:
    - A0 at origin (0, 0, 2m height)
    - A1 at (10, 0, 2m height)
    - A2 at (5, 8.66, 2m height) - forms ~60° angle

    Returns:
        Dictionary with base coordinates and anchor ENU positions.
    """
    return {
        "base_lat": 22.2900,
        "base_lon": 114.1700,
        "base_alt": 2.0,
        "anchors": {
            "A0": {"e": 0.0, "n": 0.0, "u": 2.0},
            "A1": {"e": 10.0, "n": 0.0, "u": 2.0},
            "A2": {"e": 5.0, "n": 8.66, "u": 2.0},
        },
    }


@pytest.fixture
def anchor_positions_3d() -> List[Tuple[float, float, float]]:
    """
    Standard 3D anchor positions for trilateration tests.

    Returns positions matching the virtual_gnss_config equilateral triangle.

    Returns:
        List of (x, y, z) tuples in meters.
    """
    return [
        (0.0, 0.0, 2.0),   # A0 at origin, 2m height
        (10.0, 0.0, 2.0),  # A1 at 10m east, 2m height
        (5.0, 8.66, 2.0),  # A2 at centroid-ish, 2m height
    ]


@pytest.fixture
def anchor_positions_2d() -> List[Tuple[float, float]]:
    """
    Standard 2D anchor positions for trilateration tests.

    Returns:
        List of (x, y) tuples in meters.
    """
    return [
        (0.0, 0.0),
        (10.0, 0.0),
        (5.0, 8.66),
    ]


# =============================================================================
# Coordinate Converter Fixtures
# =============================================================================


@pytest.fixture
def coordinate_converter(virtual_gnss_config: Dict) -> CoordinateConverter:
    """
    Pre-configured CoordinateConverter with virtual GNSS anchors.

    Args:
        virtual_gnss_config: Virtual GNSS configuration fixture.

    Returns:
        CoordinateConverter instance with anchors initialized.
    """
    converter = CoordinateConverter()

    # Create virtual anchor coordinates
    virtual_anchors = create_virtual_anchor_coordinates(virtual_gnss_config)

    # Convert to dictionary format expected by update_all_anchors
    gnss_dict = {}
    for anchor_id, coord in virtual_anchors.items():
        gnss_dict[anchor_id] = {
            "lat": coord.lat,
            "lon": coord.lon,
            "alt": coord.alt,
            "e": coord.e,
            "n": coord.n,
            "u": coord.u,
        }

    converter.update_all_anchors(gnss_dict)
    return converter


@pytest.fixture
def origin_gnss_coordinate() -> GNSSCoordinate:
    """
    Standard origin GNSS coordinate (Hong Kong area).

    Returns:
        GNSSCoordinate at approximately Hong Kong.
    """
    return GNSSCoordinate(
        lat=22.2900,
        lon=114.1700,
        alt=2.0,
        e=0.0,
        n=0.0,
        u=2.0,
    )


# =============================================================================
# Trilateration Fixtures
# =============================================================================


@pytest.fixture
def python_trilateration() -> PythonTrilateration:
    """
    PythonTrilateration instance for algorithm testing.

    Returns:
        PythonTrilateration instance.
    """
    return PythonTrilateration()


@pytest.fixture
def trilateration_wrapper() -> TrilaterationWrapper:
    """
    TrilaterationWrapper instance (may not have DLL loaded).

    Returns:
        TrilaterationWrapper instance.
    """
    return TrilaterationWrapper()


# =============================================================================
# Test Data Fixtures
# =============================================================================


@pytest.fixture
def known_position_test_cases() -> List[Dict]:
    """
    Known position test cases with expected results.

    Each case contains:
    - tag_position: Known (x, y, z) position of tag
    - distances_m: Expected distances to anchors in meters
    - tolerance_m: Acceptable error tolerance in meters

    Returns:
        List of test case dictionaries.
    """
    # Anchor positions: A0=(0,0,2), A1=(10,0,2), A2=(5,8.66,2)
    test_cases = [
        {
            "name": "center_ground",
            "tag_position": (5.0, 2.89, 0.0),  # Roughly center, ground level
            "distances_m": None,  # Will be calculated
            "tolerance_m": 0.5,
        },
        {
            "name": "near_A0",
            "tag_position": (1.0, 1.0, 0.0),
            "distances_m": None,
            "tolerance_m": 0.5,
        },
        {
            "name": "near_A1",
            "tag_position": (9.0, 1.0, 0.0),
            "distances_m": None,
            "tolerance_m": 0.5,
        },
        {
            "name": "elevated",
            "tag_position": (5.0, 4.0, 1.0),  # 1m above ground
            "distances_m": None,
            "tolerance_m": 0.5,
        },
    ]

    # Calculate expected distances for each test case
    anchors = [(0.0, 0.0, 2.0), (10.0, 0.0, 2.0), (5.0, 8.66, 2.0)]

    for case in test_cases:
        tx, ty, tz = case["tag_position"]
        distances = []
        for ax, ay, az in anchors:
            dist = math.sqrt((tx - ax) ** 2 + (ty - ay) ** 2 + (tz - az) ** 2)
            distances.append(dist)
        case["distances_m"] = distances

    return test_cases


@pytest.fixture
def invalid_range_test_cases() -> List[Dict]:
    """
    Test cases for invalid range handling.

    Returns:
        List of test cases with invalid ranges.
    """
    return [
        {
            "name": "below_minimum",
            "ranges_mm": [50, 5000, 6000],  # 50mm < 100mm minimum
            "expected_valid": False,
        },
        {
            "name": "above_maximum",
            "ranges_mm": [5000, 60000, 6000],  # 60000mm > 50000mm maximum
            "expected_valid": False,
        },
        {
            "name": "all_invalid",
            "ranges_mm": [-1, -1, -1],
            "expected_valid": False,
        },
        {
            "name": "all_valid",
            "ranges_mm": [5000, 6000, 7000],
            "expected_valid": True,
        },
        {
            "name": "two_valid_one_invalid",
            "ranges_mm": [5000, -1, 6000],
            "expected_valid": False,  # Need 3 anchors minimum
        },
    ]


# =============================================================================
# Helper Functions
# =============================================================================


def calculate_distance_3d(
    p1: Tuple[float, float, float], p2: Tuple[float, float, float]
) -> float:
    """
    Calculate Euclidean distance between two 3D points.

    Args:
        p1: First point (x, y, z).
        p2: Second point (x, y, z).

    Returns:
        Distance in the same units as input.
    """
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2
    )


def calculate_distance_2d(
    p1: Tuple[float, float], p2: Tuple[float, float]
) -> float:
    """
    Calculate Euclidean distance between two 2D points.

    Args:
        p1: First point (x, y).
        p2: Second point (x, y).

    Returns:
        Distance in the same units as input.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
