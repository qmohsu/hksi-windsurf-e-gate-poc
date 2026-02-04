"""
Baseline Recording Tests for HKSI UWB Positioning System.

This module records the current performance baseline before any algorithm changes.
Run this BEFORE making changes to establish ground truth.

Usage:
    pytest tests/test_baseline_record.py -v --baseline-record

Reference: Design Doc Section 7 (Localization Engine) defines accuracy requirements.
"""

import time
import math
import pytest
from typing import List, Tuple, Dict, Any

from trilateration_wrapper import PythonTrilateration
from tests.conftest import calculate_distance_2d, calculate_distance_3d
from tests.performance_baseline import (
    BaselineManager,
    TestCaseResult,
    MonteCarloTestGenerator,
    PERFORMANCE_THRESHOLDS,
    BASELINE_FILE,
)


# =============================================================================
# Pytest Configuration
# =============================================================================


def pytest_addoption(parser):
    """Add custom command line options."""
    parser.addoption(
        "--baseline-record",
        action="store_true",
        default=False,
        help="Record performance baseline",
    )


@pytest.fixture(scope="session")
def baseline_manager() -> BaselineManager:
    """Provide baseline manager instance."""
    return BaselineManager()


@pytest.fixture(scope="session")
def monte_carlo_generator(
    anchor_positions_3d: List[Tuple[float, float, float]]
) -> MonteCarloTestGenerator:
    """Provide Monte Carlo test generator."""
    return MonteCarloTestGenerator(anchor_positions_3d, seed=42)


@pytest.fixture(scope="session")
def anchor_positions_3d() -> List[Tuple[float, float, float]]:
    """Standard 3D anchor positions."""
    return [
        (0.0, 0.0, 2.0),
        (10.0, 0.0, 2.0),
        (5.0, 8.66, 2.0),
    ]


@pytest.fixture(scope="session")
def anchor_positions_2d() -> List[Tuple[float, float]]:
    """Standard 2D anchor positions."""
    return [
        (0.0, 0.0),
        (10.0, 0.0),
        (5.0, 8.66),
    ]


# =============================================================================
# Baseline Recording Tests
# =============================================================================


class TestBaselineRecord:
    """
    Tests that record performance baseline.

    These tests document the current algorithm performance to detect
    regressions or improvements after changes.
    """

    @pytest.fixture(autouse=True)
    def _setup(
        self,
        baseline_manager: BaselineManager,
        anchor_positions_2d: List[Tuple[float, float]],
        anchor_positions_3d: List[Tuple[float, float, float]],
    ):
        """Setup test fixtures."""
        self.manager = baseline_manager
        self.anchors_2d = anchor_positions_2d
        self.anchors_3d = anchor_positions_3d

    def test_record_2d_trilateration_baseline(self, request):
        """
        Record 2D trilateration accuracy baseline.

        Executes systematic test cases and records error metrics.
        """
        # Skip if not explicitly recording baseline
        if not request.config.getoption("--baseline-record", default=False):
            pytest.skip("Use --baseline-record to run this test")

        test_positions = [
            # Inside anchor triangle
            (5.0, 2.89),    # Center
            (2.0, 2.0),     # Near A0
            (8.0, 2.0),     # Near A1
            (5.0, 6.0),     # Near A2
            # On triangle edges
            (5.0, 0.0),     # Between A0-A1
            (2.5, 4.33),    # Between A0-A2
            (7.5, 4.33),    # Between A1-A2
            # Outside triangle
            (-2.0, -2.0),   # Far outside
            (12.0, 0.0),    # Past A1
            (5.0, 12.0),    # Past A2
            (0.0, 5.0),     # Left side
            (10.0, 5.0),    # Right side
        ]

        results = []

        for ground_truth in test_positions:
            # Calculate true distances
            distances = [
                calculate_distance_2d(ground_truth, anchor)
                for anchor in self.anchors_2d
            ]

            # Time the solve
            start = time.perf_counter()
            result = PythonTrilateration.trilaterate_2d(self.anchors_2d, distances)
            solve_time_ms = (time.perf_counter() - start) * 1000

            if result is not None:
                error = calculate_distance_2d(result, ground_truth)
                results.append(
                    TestCaseResult(
                        name=f"2d_pos_{ground_truth[0]:.1f}_{ground_truth[1]:.1f}",
                        ground_truth=(ground_truth[0], ground_truth[1], 0.0),
                        estimated_position=(result[0], result[1], 0.0),
                        error_m=error,
                        solve_time_ms=solve_time_ms,
                        success=True,
                        anchors_used=3,
                    )
                )
            else:
                results.append(
                    TestCaseResult(
                        name=f"2d_pos_{ground_truth[0]:.1f}_{ground_truth[1]:.1f}",
                        ground_truth=(ground_truth[0], ground_truth[1], 0.0),
                        estimated_position=None,
                        error_m=float("inf"),
                        solve_time_ms=solve_time_ms,
                        success=False,
                        anchors_used=3,
                    )
                )

        # Record baseline
        metrics = self.manager.record_baseline(results, algorithm_version="2d_baseline")
        self.manager.save_baseline(metrics)

        # Report summary
        print(f"\n{'='*60}")
        print("2D TRILATERATION BASELINE RECORDED")
        print(f"{'='*60}")
        print(f"  Tests:         {metrics.test_count}")
        print(f"  Success Rate:  {metrics.success_rate*100:.1f}%")
        print(f"  RMSE:          {metrics.rmse_m:.4f} m")
        print(f"  Max Error:     {metrics.max_error_m:.4f} m")
        print(f"  95th %ile:     {metrics.percentile_95_m:.4f} m")
        print(f"  Mean Time:     {metrics.mean_solve_time_ms:.3f} ms")
        print(f"  Saved to:      {BASELINE_FILE}")
        print(f"{'='*60}")

    def test_record_3d_trilateration_baseline(self, request):
        """
        Record 3D trilateration accuracy baseline.

        Tests positions at various heights.
        """
        if not request.config.getoption("--baseline-record", default=False):
            pytest.skip("Use --baseline-record to run this test")

        test_positions = [
            # Ground level (z=0)
            (5.0, 2.89, 0.0),
            (2.0, 2.0, 0.0),
            (8.0, 2.0, 0.0),
            # Elevated (z=1)
            (5.0, 2.89, 1.0),
            (2.0, 2.0, 1.0),
            (8.0, 2.0, 1.0),
            # At anchor height (z=2)
            (5.0, 4.0, 2.0),
            (3.0, 3.0, 2.0),
            # Outside triangle at various heights
            (-2.0, -2.0, 0.0),
            (-2.0, -2.0, 1.0),
            (12.0, 5.0, 0.5),
        ]

        results = []

        for ground_truth in test_positions:
            distances = [
                calculate_distance_3d(ground_truth, anchor)
                for anchor in self.anchors_3d
            ]

            start = time.perf_counter()
            result = PythonTrilateration.trilaterate_3d(self.anchors_3d, distances)
            solve_time_ms = (time.perf_counter() - start) * 1000

            if result is not None:
                error = calculate_distance_3d(result, ground_truth)
                # For 3D, also compute XY error separately
                xy_error = calculate_distance_2d(
                    (result[0], result[1]), (ground_truth[0], ground_truth[1])
                )
                results.append(
                    TestCaseResult(
                        name=f"3d_pos_{ground_truth[0]:.1f}_{ground_truth[1]:.1f}_{ground_truth[2]:.1f}",
                        ground_truth=ground_truth,
                        estimated_position=result,
                        error_m=error,
                        solve_time_ms=solve_time_ms,
                        success=True,
                        anchors_used=3,
                    )
                )
            else:
                results.append(
                    TestCaseResult(
                        name=f"3d_pos_{ground_truth[0]:.1f}_{ground_truth[1]:.1f}_{ground_truth[2]:.1f}",
                        ground_truth=ground_truth,
                        estimated_position=None,
                        error_m=float("inf"),
                        solve_time_ms=solve_time_ms,
                        success=False,
                        anchors_used=3,
                    )
                )

        metrics = self.manager.record_baseline(results, algorithm_version="3d_baseline")

        # Save to separate file for 3D
        from pathlib import Path

        baseline_3d_path = Path(BASELINE_FILE).parent / "performance_baseline_3d.json"
        manager_3d = BaselineManager(baseline_3d_path)
        manager_3d.save_baseline(metrics)

        print(f"\n{'='*60}")
        print("3D TRILATERATION BASELINE RECORDED")
        print(f"{'='*60}")
        print(f"  Tests:         {metrics.test_count}")
        print(f"  Success Rate:  {metrics.success_rate*100:.1f}%")
        print(f"  RMSE:          {metrics.rmse_m:.4f} m")
        print(f"  Max Error:     {metrics.max_error_m:.4f} m")
        print(f"  95th %ile:     {metrics.percentile_95_m:.4f} m")
        print(f"  Saved to:      {baseline_3d_path}")
        print(f"{'='*60}")

    def test_record_noise_robustness_baseline(
        self,
        request,
        monte_carlo_generator: MonteCarloTestGenerator,
    ):
        """
        Record baseline performance under noisy conditions.

        Uses Monte Carlo simulation with ±5cm distance noise.
        """
        if not request.config.getoption("--baseline-record", default=False):
            pytest.skip("Use --baseline-record to run this test")

        # Generate 100 random test cases
        test_cases = monte_carlo_generator.generate_test_cases(
            count=100, noise_std_m=0.05
        )

        results = []

        for case in test_cases:
            gt = case["ground_truth"]
            distances = case["noisy_distances_m"]

            # Use 2D trilateration (ignore Z for now)
            anchors_2d = [(a[0], a[1]) for a in self.anchors_3d]
            distances_2d = [
                math.sqrt(distances[i] ** 2 - (gt[2] - self.anchors_3d[i][2]) ** 2)
                if distances[i] ** 2 > (gt[2] - self.anchors_3d[i][2]) ** 2
                else distances[i]
                for i in range(len(distances))
            ]

            start = time.perf_counter()
            result = PythonTrilateration.trilaterate_2d(anchors_2d, distances_2d)
            solve_time_ms = (time.perf_counter() - start) * 1000

            if result is not None:
                error = calculate_distance_2d(result, (gt[0], gt[1]))
                results.append(
                    TestCaseResult(
                        name=case["name"],
                        ground_truth=gt,
                        estimated_position=(result[0], result[1], gt[2]),
                        error_m=error,
                        solve_time_ms=solve_time_ms,
                        success=True,
                    )
                )
            else:
                results.append(
                    TestCaseResult(
                        name=case["name"],
                        ground_truth=gt,
                        estimated_position=None,
                        error_m=float("inf"),
                        solve_time_ms=solve_time_ms,
                        success=False,
                    )
                )

        metrics = self.manager.record_baseline(
            results, algorithm_version="noise_5cm_baseline"
        )

        # Save to separate file
        from pathlib import Path

        baseline_path = Path(BASELINE_FILE).parent / "performance_baseline_noise.json"
        manager = BaselineManager(baseline_path)
        manager.save_baseline(metrics)

        print(f"\n{'='*60}")
        print("NOISE ROBUSTNESS BASELINE RECORDED (±5cm)")
        print(f"{'='*60}")
        print(f"  Tests:         {metrics.test_count}")
        print(f"  Success Rate:  {metrics.success_rate*100:.1f}%")
        print(f"  RMSE:          {metrics.rmse_m:.4f} m")
        print(f"  Max Error:     {metrics.max_error_m:.4f} m")
        print(f"  95th %ile:     {metrics.percentile_95_m:.4f} m")
        print(f"  Threshold:     {PERFORMANCE_THRESHOLDS['robustness']['noise_tolerance_5cm']:.2f} m")
        print(f"  Saved to:      {baseline_path}")
        print(f"{'='*60}")

        # Verify against threshold
        assert metrics.percentile_95_m < PERFORMANCE_THRESHOLDS["robustness"][
            "noise_tolerance_5cm"
        ], (
            f"95th percentile error {metrics.percentile_95_m:.4f}m exceeds "
            f"threshold {PERFORMANCE_THRESHOLDS['robustness']['noise_tolerance_5cm']:.2f}m"
        )


# =============================================================================
# Quick Baseline Check (No Recording)
# =============================================================================


class TestBaselineQuickCheck:
    """
    Quick check tests that don't record baseline.

    Run these to verify current algorithm meets thresholds.
    """

    def test_2d_accuracy_meets_threshold(
        self, anchor_positions_2d: List[Tuple[float, float]]
    ):
        """Verify 2D trilateration meets accuracy threshold."""
        # Test at center of triangle
        ground_truth = (5.0, 2.89)
        distances = [
            calculate_distance_2d(ground_truth, anchor)
            for anchor in anchor_positions_2d
        ]

        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)

        assert result is not None, "Trilateration failed"
        error = calculate_distance_2d(result, ground_truth)
        assert error < PERFORMANCE_THRESHOLDS["accuracy"]["center_tolerance_m"], (
            f"Error {error:.4f}m exceeds threshold "
            f"{PERFORMANCE_THRESHOLDS['accuracy']['center_tolerance_m']:.2f}m"
        )

    def test_3d_accuracy_meets_threshold(
        self, anchor_positions_3d: List[Tuple[float, float, float]]
    ):
        """Verify 3D trilateration meets accuracy threshold."""
        ground_truth = (5.0, 2.89, 0.0)
        distances = [
            calculate_distance_3d(ground_truth, anchor)
            for anchor in anchor_positions_3d
        ]

        result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances)

        assert result is not None, "Trilateration failed"
        # Check XY error (Z has larger tolerance)
        xy_error = calculate_distance_2d(
            (result[0], result[1]), (ground_truth[0], ground_truth[1])
        )
        assert xy_error < PERFORMANCE_THRESHOLDS["accuracy"]["center_tolerance_m"] * 2, (
            f"XY error {xy_error:.4f}m exceeds threshold"
        )
