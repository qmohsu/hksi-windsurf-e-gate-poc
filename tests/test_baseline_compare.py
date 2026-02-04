"""
Baseline Comparison Tests for HKSI UWB Positioning System.

This module compares current algorithm performance against recorded baseline
to detect improvements or regressions.

Usage:
    # First, record baseline (before changes)
    pytest tests/test_baseline_record.py -v --baseline-record

    # Then, after changes, run comparison
    pytest tests/test_baseline_compare.py -v

Reference: Design Doc Section 7 (Localization Engine) defines accuracy requirements.
"""

import time
import math
import pytest
from pathlib import Path
from typing import List, Tuple

from trilateration_wrapper import PythonTrilateration
from tests.conftest import calculate_distance_2d, calculate_distance_3d
from tests.performance_baseline import (
    BaselineManager,
    BaselineMetrics,
    TestCaseResult,
    ComparisonResult,
    MonteCarloTestGenerator,
    PERFORMANCE_THRESHOLDS,
    BASELINE_FILE,
)


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture(scope="session")
def baseline_manager() -> BaselineManager:
    """Provide baseline manager instance."""
    return BaselineManager()


@pytest.fixture(scope="session")
def recorded_baseline(baseline_manager: BaselineManager) -> BaselineMetrics:
    """Load recorded baseline, skip if not available."""
    baseline = baseline_manager.load_baseline()
    if baseline is None:
        pytest.skip(
            "No baseline recorded. Run: pytest tests/test_baseline_record.py --baseline-record"
        )
    return baseline


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


@pytest.fixture(scope="session")
def monte_carlo_generator(
    anchor_positions_3d: List[Tuple[float, float, float]]
) -> MonteCarloTestGenerator:
    """Provide Monte Carlo test generator with fixed seed."""
    return MonteCarloTestGenerator(anchor_positions_3d, seed=42)


# =============================================================================
# Comparison Tests
# =============================================================================


class TestBaselineComparison:
    """
    Tests that compare current performance against recorded baseline.

    These tests will PASS if performance is maintained or improved,
    and FAIL if significant degradation is detected.
    """

    def test_compare_2d_trilateration(
        self,
        baseline_manager: BaselineManager,
        recorded_baseline: BaselineMetrics,
        anchor_positions_2d: List[Tuple[float, float]],
    ):
        """
        Compare current 2D trilateration against baseline.

        Runs the same test cases from the baseline and compares metrics.
        """
        # Extract test positions from recorded baseline
        results = []

        for recorded_result in recorded_baseline.test_results:
            if not recorded_result["name"].startswith("2d_pos_"):
                continue

            ground_truth = recorded_result["ground_truth"]
            gt_2d = (ground_truth[0], ground_truth[1])

            # Calculate distances
            distances = [
                calculate_distance_2d(gt_2d, anchor) for anchor in anchor_positions_2d
            ]

            # Run current algorithm
            start = time.perf_counter()
            result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)
            solve_time_ms = (time.perf_counter() - start) * 1000

            if result is not None:
                error = calculate_distance_2d(result, gt_2d)
                results.append(
                    TestCaseResult(
                        name=recorded_result["name"],
                        ground_truth=ground_truth,
                        estimated_position=(result[0], result[1], 0.0),
                        error_m=error,
                        solve_time_ms=solve_time_ms,
                        success=True,
                    )
                )
            else:
                results.append(
                    TestCaseResult(
                        name=recorded_result["name"],
                        ground_truth=ground_truth,
                        estimated_position=None,
                        error_m=float("inf"),
                        solve_time_ms=solve_time_ms,
                        success=False,
                    )
                )

        # Skip if no matching test cases
        if not results:
            pytest.skip("No 2D test cases found in baseline")

        # Calculate current metrics
        current_metrics = baseline_manager.record_baseline(
            results, algorithm_version="current"
        )

        # Compare
        comparisons = baseline_manager.compare_baselines(
            recorded_baseline, current_metrics
        )

        # Generate report
        report = baseline_manager.generate_report(comparisons)
        print(f"\n{report}")

        # Check for degradations
        degradations = [c for c in comparisons if c.verdict == "DEGRADATION"]

        if degradations:
            degradation_msgs = [
                f"  - {c.metric_name}: {c.baseline_value:.4f} → {c.current_value:.4f} "
                f"({c.delta_percent:+.2f}%)"
                for c in degradations
            ]
            pytest.fail(
                f"Performance degradation detected:\n" + "\n".join(degradation_msgs)
            )

    def test_compare_noise_robustness(
        self,
        baseline_manager: BaselineManager,
        anchor_positions_2d: List[Tuple[float, float]],
        anchor_positions_3d: List[Tuple[float, float, float]],
        monte_carlo_generator: MonteCarloTestGenerator,
    ):
        """
        Compare noise robustness against baseline.

        Uses same Monte Carlo seed for reproducibility.
        """
        # Load noise baseline
        noise_baseline_path = Path(BASELINE_FILE).parent / "performance_baseline_noise.json"
        noise_manager = BaselineManager(noise_baseline_path)
        noise_baseline = noise_manager.load_baseline()

        if noise_baseline is None:
            pytest.skip(
                "No noise baseline recorded. "
                "Run: pytest tests/test_baseline_record.py --baseline-record"
            )

        # Generate same test cases (fixed seed ensures reproducibility)
        test_cases = monte_carlo_generator.generate_test_cases(
            count=100, noise_std_m=0.05
        )

        results = []

        for case in test_cases:
            gt = case["ground_truth"]
            distances = case["noisy_distances_m"]

            # Use 2D trilateration
            distances_2d = [
                math.sqrt(distances[i] ** 2 - (gt[2] - anchor_positions_3d[i][2]) ** 2)
                if distances[i] ** 2 > (gt[2] - anchor_positions_3d[i][2]) ** 2
                else distances[i]
                for i in range(len(distances))
            ]

            start = time.perf_counter()
            result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances_2d)
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

        # Calculate current metrics
        current_metrics = noise_manager.record_baseline(results, algorithm_version="current")

        # Compare
        comparisons = noise_manager.compare_baselines(noise_baseline, current_metrics)

        # Generate report
        report = noise_manager.generate_report(comparisons)
        print(f"\n{report}")

        # Check for degradations
        degradations = [c for c in comparisons if c.verdict == "DEGRADATION"]

        if degradations:
            degradation_msgs = [
                f"  - {c.metric_name}: {c.baseline_value:.4f} → {c.current_value:.4f} "
                f"({c.delta_percent:+.2f}%)"
                for c in degradations
            ]
            pytest.fail(
                f"Noise robustness degradation detected:\n" + "\n".join(degradation_msgs)
            )


# =============================================================================
# Statistical Comparison Tests
# =============================================================================


class TestStatisticalComparison:
    """
    Statistical tests for algorithm comparison.

    Uses paired t-test to determine if differences are significant.
    """

    def test_paired_comparison_monte_carlo(
        self,
        anchor_positions_2d: List[Tuple[float, float]],
        anchor_positions_3d: List[Tuple[float, float, float]],
        monte_carlo_generator: MonteCarloTestGenerator,
    ):
        """
        Paired statistical comparison using Monte Carlo simulation.

        Compares old vs new algorithm on identical inputs.
        Currently both are the same (PythonTrilateration), so this
        serves as a template for when a new algorithm is implemented.
        """
        # Generate test cases
        test_cases = monte_carlo_generator.generate_test_cases(count=100)

        old_errors = []
        new_errors = []

        for case in test_cases:
            gt = case["ground_truth"]
            gt_2d = (gt[0], gt[1])
            distances = case["noisy_distances_m"]

            # Calculate 2D distances
            distances_2d = [
                math.sqrt(distances[i] ** 2 - (gt[2] - anchor_positions_3d[i][2]) ** 2)
                if distances[i] ** 2 > (gt[2] - anchor_positions_3d[i][2]) ** 2
                else distances[i]
                for i in range(len(distances))
            ]

            # Old algorithm (current implementation)
            old_result = PythonTrilateration.trilaterate_2d(
                anchor_positions_2d, distances_2d
            )
            if old_result:
                old_errors.append(calculate_distance_2d(old_result, gt_2d))
            else:
                old_errors.append(float("inf"))

            # New algorithm (placeholder - same as old for now)
            # Replace this with new algorithm when implemented
            new_result = PythonTrilateration.trilaterate_2d(
                anchor_positions_2d, distances_2d
            )
            if new_result:
                new_errors.append(calculate_distance_2d(new_result, gt_2d))
            else:
                new_errors.append(float("inf"))

        # Filter out infinite errors for statistical test
        paired_errors = [
            (old, new)
            for old, new in zip(old_errors, new_errors)
            if old != float("inf") and new != float("inf")
        ]

        if len(paired_errors) < 10:
            pytest.skip("Not enough valid results for statistical comparison")

        old_filtered = [p[0] for p in paired_errors]
        new_filtered = [p[1] for p in paired_errors]

        # Calculate statistics
        old_mean = sum(old_filtered) / len(old_filtered)
        new_mean = sum(new_filtered) / len(new_filtered)
        mean_diff = new_mean - old_mean

        # Simple paired difference test (without scipy dependency)
        differences = [new - old for old, new in paired_errors]
        diff_mean = sum(differences) / len(differences)
        diff_var = sum((d - diff_mean) ** 2 for d in differences) / (len(differences) - 1)
        diff_std = math.sqrt(diff_var) if diff_var > 0 else 0.001

        # t-statistic
        t_stat = diff_mean / (diff_std / math.sqrt(len(differences)))

        # Report
        print(f"\n{'='*60}")
        print("STATISTICAL COMPARISON")
        print(f"{'='*60}")
        print(f"  Samples:     {len(paired_errors)}")
        print(f"  Old Mean:    {old_mean:.4f} m")
        print(f"  New Mean:    {new_mean:.4f} m")
        print(f"  Difference:  {mean_diff:+.4f} m ({mean_diff/old_mean*100:+.2f}%)")
        print(f"  t-statistic: {t_stat:.3f}")
        print(f"{'='*60}")

        # Interpret: t > 2 suggests significant increase (worse)
        #            t < -2 suggests significant decrease (better)
        if t_stat > 2.0:
            verdict = "DEGRADATION (statistically significant)"
        elif t_stat < -2.0:
            verdict = "IMPROVEMENT (statistically significant)"
        else:
            verdict = "NO SIGNIFICANT CHANGE"

        print(f"  Verdict:     {verdict}")
        print(f"{'='*60}")

        # Fail on degradation
        assert t_stat <= 2.0, f"Statistically significant degradation detected: t={t_stat:.3f}"


# =============================================================================
# Threshold Compliance Tests
# =============================================================================


class TestThresholdCompliance:
    """
    Tests that verify algorithm meets defined performance thresholds.

    These are absolute tests that don't depend on a recorded baseline.
    """

    def test_rmse_threshold(
        self,
        anchor_positions_2d: List[Tuple[float, float]],
        monte_carlo_generator: MonteCarloTestGenerator,
    ):
        """Verify RMSE meets threshold across Monte Carlo tests."""
        test_cases = monte_carlo_generator.generate_test_cases(count=50)
        errors = []

        for case in test_cases:
            gt_2d = (case["ground_truth"][0], case["ground_truth"][1])
            distances = case["true_distances_m"]  # Use true distances for this test
            distances_2d = distances[:3]  # First 3 anchors

            result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances_2d)
            if result:
                error = calculate_distance_2d(result, gt_2d)
                errors.append(error)

        if not errors:
            pytest.fail("All trilateration attempts failed")

        rmse = math.sqrt(sum(e ** 2 for e in errors) / len(errors))
        threshold = PERFORMANCE_THRESHOLDS["accuracy"]["max_rmse_m"]

        print(f"\n  RMSE: {rmse:.4f} m (threshold: {threshold:.2f} m)")

        assert rmse < threshold, f"RMSE {rmse:.4f}m exceeds threshold {threshold:.2f}m"

    def test_success_rate_threshold(
        self,
        anchor_positions_2d: List[Tuple[float, float]],
        monte_carlo_generator: MonteCarloTestGenerator,
    ):
        """Verify success rate meets threshold."""
        test_cases = monte_carlo_generator.generate_test_cases(count=100)
        successes = 0

        for case in test_cases:
            distances = case["true_distances_m"][:3]
            result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)
            if result is not None:
                successes += 1

        success_rate = successes / len(test_cases)
        threshold = PERFORMANCE_THRESHOLDS["reliability"]["min_solve_success_rate"]

        print(f"\n  Success Rate: {success_rate*100:.1f}% (threshold: {threshold*100:.0f}%)")

        assert success_rate >= threshold, (
            f"Success rate {success_rate*100:.1f}% below threshold {threshold*100:.0f}%"
        )
