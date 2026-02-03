"""
Unit tests for trilateration algorithms.

Tests cover:
- 2D trilateration (PythonTrilateration.trilaterate_2d)
- 3D trilateration (PythonTrilateration.trilaterate_3d)
- TrilaterationWrapper anchor/distance setting
- Edge cases: singular matrices, insufficient anchors, invalid distances

Reference: These tests establish baseline behavior before refactoring.
"""

import math
from typing import List, Tuple

import pytest

from trilateration_wrapper import (
    TrilaterationWrapper,
    PythonTrilateration,
    Vec3D,
    UWBMsg,
)
from tests.conftest import calculate_distance_2d, calculate_distance_3d


class TestPythonTrilateration2D:
    """Tests for 2D trilateration algorithm."""

    def test_trilaterate_2d_known_point_at_origin(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """
        Test 2D trilateration with tag at origin.

        The tag is at (0, 0), distances calculated from anchor positions.
        """
        tag_pos = (0.0, 0.0)
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, "Trilateration should succeed"
        assert abs(result[0] - tag_pos[0]) < 0.01, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.01, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_2d_known_point_at_center(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """
        Test 2D trilateration with tag at approximate centroid.

        Centroid of equilateral triangle with vertices at:
        (0,0), (10,0), (5, 8.66) is approximately (5, 2.89).
        """
        tag_pos = (5.0, 2.89)
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, "Trilateration should succeed"
        assert abs(result[0] - tag_pos[0]) < 0.1, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.1, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_2d_near_anchor_a0(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """Test 2D trilateration with tag near anchor A0."""
        tag_pos = (1.0, 1.0)
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, "Trilateration should succeed"
        assert abs(result[0] - tag_pos[0]) < 0.1, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.1, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_2d_near_anchor_a1(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """Test 2D trilateration with tag near anchor A1."""
        tag_pos = (9.0, 1.0)
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, "Trilateration should succeed"
        assert abs(result[0] - tag_pos[0]) < 0.1, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.1, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_2d_outside_triangle(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """Test 2D trilateration with tag outside the anchor triangle."""
        tag_pos = (-2.0, -2.0)  # Outside the triangle
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, "Trilateration should succeed even outside triangle"
        assert abs(result[0] - tag_pos[0]) < 0.1, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.1, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_2d_collinear_anchors_fails(self):
        """
        Test that 2D trilateration fails gracefully with collinear anchors.

        Collinear anchors create a singular matrix (det ≈ 0).
        """
        # Three collinear anchors on x-axis
        collinear_anchors = [
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
        ]
        distances = [3.0, 3.0, 3.0]  # Arbitrary distances

        result = PythonTrilateration.trilaterate_2d(collinear_anchors, distances)

        # Should return None due to singular matrix
        assert result is None, "Collinear anchors should fail gracefully"

    def test_trilaterate_2d_insufficient_anchors(self):
        """Test that 2D trilateration fails with fewer than 3 anchors."""
        anchors_2 = [(0.0, 0.0), (10.0, 0.0)]
        distances_2 = [5.0, 5.0]

        result = PythonTrilateration.trilaterate_2d(anchors_2, distances_2)

        assert result is None, "Should fail with only 2 anchors"

    def test_trilaterate_2d_empty_input(self):
        """Test that 2D trilateration handles empty input."""
        result = PythonTrilateration.trilaterate_2d([], [])
        assert result is None, "Should fail with empty input"


class TestPythonTrilateration3D:
    """Tests for 3D trilateration algorithm."""

    def test_trilaterate_3d_known_point_ground_level(
        self, anchor_positions_3d: List[Tuple[float, float, float]]
    ):
        """
        Test 3D trilateration with tag at ground level (z=0).

        Tag at (5, 2.89, 0) - center of triangle at ground level.
        """
        tag_pos = (5.0, 2.89, 0.0)
        distances = [
            calculate_distance_3d(tag_pos, anchor_positions_3d[0]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[1]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[2]),
        ]

        result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances)

        assert result is not None, "3D trilateration should succeed"
        assert abs(result[0] - tag_pos[0]) < 0.5, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.5, f"Y error: {result[1]} vs {tag_pos[1]}"
        # Z tolerance is larger due to algorithm limitations
        assert abs(result[2] - tag_pos[2]) < 1.0, f"Z error: {result[2]} vs {tag_pos[2]}"

    def test_trilaterate_3d_elevated_tag(
        self, anchor_positions_3d: List[Tuple[float, float, float]]
    ):
        """Test 3D trilateration with elevated tag (z=1m)."""
        tag_pos = (5.0, 4.0, 1.0)
        distances = [
            calculate_distance_3d(tag_pos, anchor_positions_3d[0]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[1]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[2]),
        ]

        result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances)

        assert result is not None, "3D trilateration should succeed"
        # X and Y should be accurate
        assert abs(result[0] - tag_pos[0]) < 0.5, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.5, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_3d_tag_at_anchor_height(
        self, anchor_positions_3d: List[Tuple[float, float, float]]
    ):
        """Test 3D trilateration with tag at same height as anchors."""
        tag_pos = (5.0, 4.0, 2.0)  # Same height as anchors
        distances = [
            calculate_distance_3d(tag_pos, anchor_positions_3d[0]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[1]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[2]),
        ]

        result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances)

        assert result is not None, "3D trilateration should succeed"
        assert abs(result[0] - tag_pos[0]) < 0.5, f"X error: {result[0]} vs {tag_pos[0]}"
        assert abs(result[1] - tag_pos[1]) < 0.5, f"Y error: {result[1]} vs {tag_pos[1]}"

    def test_trilaterate_3d_insufficient_anchors(self):
        """Test that 3D trilateration fails with fewer than 3 anchors."""
        anchors = [(0.0, 0.0, 2.0), (10.0, 0.0, 2.0)]
        distances = [5.0, 5.0]

        result = PythonTrilateration.trilaterate_3d(anchors, distances)

        assert result is None, "Should fail with only 2 anchors"

    def test_trilaterate_3d_with_known_test_cases(
        self,
        anchor_positions_3d: List[Tuple[float, float, float]],
        known_position_test_cases: List[dict],
    ):
        """Test 3D trilateration with predefined known positions."""
        for case in known_position_test_cases:
            tag_pos = case["tag_position"]
            distances = case["distances_m"]
            tolerance = case["tolerance_m"]

            result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances)

            assert result is not None, f"Case '{case['name']}' should succeed"
            xy_error = math.sqrt(
                (result[0] - tag_pos[0]) ** 2 + (result[1] - tag_pos[1]) ** 2
            )
            assert xy_error < tolerance, (
                f"Case '{case['name']}' XY error {xy_error:.3f}m exceeds tolerance {tolerance}m"
            )


class TestTrilaterationWrapper:
    """Tests for TrilaterationWrapper class."""

    def test_wrapper_initialization(self, trilateration_wrapper: TrilaterationWrapper):
        """Test that wrapper initializes with correct defaults."""
        assert trilateration_wrapper.MAX_ANCHORS == 8
        assert trilateration_wrapper.dll is None  # DLL not loaded yet
        assert len(trilateration_wrapper.anchor_array) == 8
        assert len(trilateration_wrapper.distance_array) == 8

    def test_set_single_anchor_position(
        self, trilateration_wrapper: TrilaterationWrapper
    ):
        """Test setting a single anchor position."""
        trilateration_wrapper.set_anchor_position(0, 1.0, 2.0, 3.0)

        assert trilateration_wrapper.anchor_array[0].x == 1.0
        assert trilateration_wrapper.anchor_array[0].y == 2.0
        assert trilateration_wrapper.anchor_array[0].z == 3.0

    def test_set_anchor_positions_batch(
        self,
        trilateration_wrapper: TrilaterationWrapper,
        anchor_positions_3d: List[Tuple[float, float, float]],
    ):
        """Test batch setting of anchor positions."""
        trilateration_wrapper.set_anchor_positions(anchor_positions_3d)

        for i, (x, y, z) in enumerate(anchor_positions_3d):
            assert trilateration_wrapper.anchor_array[i].x == x
            assert trilateration_wrapper.anchor_array[i].y == y
            assert trilateration_wrapper.anchor_array[i].z == z

    def test_set_anchor_position_invalid_index(
        self, trilateration_wrapper: TrilaterationWrapper
    ):
        """Test that invalid anchor index is handled gracefully."""
        # Should not raise exception, just log warning
        trilateration_wrapper.set_anchor_position(10, 1.0, 2.0, 3.0)  # Index > 7
        trilateration_wrapper.set_anchor_position(-1, 1.0, 2.0, 3.0)  # Negative index

    def test_set_single_distance(self, trilateration_wrapper: TrilaterationWrapper):
        """Test setting a single distance value."""
        trilateration_wrapper.set_distance(0, 5000)  # 5000mm = 5m

        assert trilateration_wrapper.distance_array[0] == 5000

    def test_set_distances_batch(self, trilateration_wrapper: TrilaterationWrapper):
        """Test batch setting of distances."""
        distances = [1000, 2000, 3000, -1, -1, -1, -1, -1]
        trilateration_wrapper.set_distances(distances)

        assert trilateration_wrapper.distance_array[0] == 1000
        assert trilateration_wrapper.distance_array[1] == 2000
        assert trilateration_wrapper.distance_array[2] == 3000
        assert trilateration_wrapper.distance_array[3] == -1

    def test_set_distances_clears_previous(
        self, trilateration_wrapper: TrilaterationWrapper
    ):
        """Test that set_distances clears previous values."""
        # Set initial distances
        trilateration_wrapper.set_distances([1000, 2000, 3000])

        # Set new distances (shorter list)
        trilateration_wrapper.set_distances([4000, 5000])

        assert trilateration_wrapper.distance_array[0] == 4000
        assert trilateration_wrapper.distance_array[1] == 5000
        assert trilateration_wrapper.distance_array[2] == -1  # Should be cleared

    def test_set_distances_invalid_values_converted(
        self, trilateration_wrapper: TrilaterationWrapper
    ):
        """Test that zero and negative distances are converted to -1."""
        distances = [1000, 0, -500, 2000]
        trilateration_wrapper.set_distances(distances)

        assert trilateration_wrapper.distance_array[0] == 1000
        assert trilateration_wrapper.distance_array[1] == -1  # 0 -> -1
        assert trilateration_wrapper.distance_array[2] == -1  # -500 -> -1
        assert trilateration_wrapper.distance_array[3] == 2000

    def test_calculate_location_without_dll_returns_none(
        self, trilateration_wrapper: TrilaterationWrapper
    ):
        """Test that calculate_location returns None when DLL not loaded."""
        result = trilateration_wrapper.calculate_location()
        assert result is None

    def test_load_dll_nonexistent_path(
        self, trilateration_wrapper: TrilaterationWrapper
    ):
        """Test that loading DLL from nonexistent path fails gracefully."""
        result = trilateration_wrapper.load_dll("/nonexistent/path/trilateration.dll")
        assert result is False


class TestVec3D:
    """Tests for Vec3D dataclass."""

    def test_vec3d_creation(self):
        """Test Vec3D creation with values."""
        vec = Vec3D(x=1.0, y=2.0, z=3.0)

        assert vec.x == 1.0
        assert vec.y == 2.0
        assert vec.z == 3.0

    def test_vec3d_equality(self):
        """Test Vec3D equality comparison."""
        vec1 = Vec3D(x=1.0, y=2.0, z=3.0)
        vec2 = Vec3D(x=1.0, y=2.0, z=3.0)
        vec3 = Vec3D(x=1.0, y=2.0, z=4.0)

        assert vec1 == vec2
        assert vec1 != vec3


class TestUWBMsg:
    """Tests for UWBMsg ctypes structure."""

    def test_uwbmsg_creation(self):
        """Test UWBMsg structure creation."""
        msg = UWBMsg()
        msg.x = 1.5
        msg.y = 2.5
        msg.z = 3.5

        assert msg.x == 1.5
        assert msg.y == 2.5
        assert msg.z == 3.5

    def test_uwbmsg_fields(self):
        """Test UWBMsg has correct fields."""
        field_names = [f[0] for f in UWBMsg._fields_]

        assert "x" in field_names
        assert "y" in field_names
        assert "z" in field_names


class TestTrilaterationAccuracy:
    """
    Accuracy benchmark tests for trilateration algorithms.

    These tests document the expected accuracy of the current implementation
    to ensure refactoring doesn't degrade performance.
    """

    @pytest.mark.parametrize(
        "tag_x,tag_y,expected_error_m",
        [
            (0.0, 0.0, 0.1),      # At anchor A0
            (5.0, 2.89, 0.1),     # Center of triangle
            (10.0, 0.0, 0.1),     # At anchor A1
            (5.0, 8.66, 0.1),     # At anchor A2
            (-5.0, -5.0, 0.5),    # Far outside triangle
            (15.0, 10.0, 0.5),    # Far outside triangle
        ],
    )
    def test_trilaterate_2d_accuracy(
        self,
        anchor_positions_2d: List[Tuple[float, float]],
        tag_x: float,
        tag_y: float,
        expected_error_m: float,
    ):
        """
        Parameterized test for 2D trilateration accuracy at various positions.

        Args:
            anchor_positions_2d: Anchor positions fixture.
            tag_x: Tag X coordinate.
            tag_y: Tag Y coordinate.
            expected_error_m: Maximum acceptable error in meters.
        """
        tag_pos = (tag_x, tag_y)
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, f"Trilateration failed for tag at ({tag_x}, {tag_y})"

        error = calculate_distance_2d(result, tag_pos)
        assert error < expected_error_m, (
            f"Error {error:.4f}m exceeds tolerance {expected_error_m}m "
            f"for tag at ({tag_x}, {tag_y})"
        )

    def test_trilaterate_with_noisy_distances(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """
        Test trilateration robustness with slightly noisy distance measurements.

        Simulates real-world UWB measurement noise (±5cm).
        """
        import random

        random.seed(42)  # Reproducible results

        tag_pos = (5.0, 4.0)
        base_distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]

        # Add ±5cm noise
        noisy_distances = [d + random.uniform(-0.05, 0.05) for d in base_distances]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, noisy_distances)

        assert result is not None, "Should handle noisy distances"

        error = calculate_distance_2d(result, tag_pos)
        # With 5cm noise, expect error < 20cm
        assert error < 0.20, f"Error {error:.4f}m too large for noisy input"
