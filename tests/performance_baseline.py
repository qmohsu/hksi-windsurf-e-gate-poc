"""
Performance Baseline Module for HKSI UWB Positioning System.

This module provides utilities for recording, loading, and comparing
performance baselines to detect improvements or regressions.

Usage:
    # Record baseline before refactoring
    pytest tests/test_baseline_record.py --baseline-record

    # Compare against baseline after changes
    pytest tests/test_baseline_compare.py

Reference: Design Doc Section 7 (Localization Engine) defines accuracy requirements.
"""

import json
import math
import subprocess
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

# =============================================================================
# Constants and Configuration
# =============================================================================

BASELINE_DIR = Path(__file__).parent / "baselines"
BASELINE_FILE = BASELINE_DIR / "performance_baseline.json"

# Performance thresholds based on design_doc.md requirements
PERFORMANCE_THRESHOLDS = {
    "accuracy": {
        "max_rmse_m": 0.15,              # Root mean square error
        "max_95th_percentile_m": 0.25,   # 95% of errors below this
        "max_error_m": 0.50,             # Absolute worst case
        "center_tolerance_m": 0.10,      # Tolerance at center of anchor triangle
        "edge_tolerance_m": 0.50,        # Tolerance outside anchor triangle
    },
    "robustness": {
        "outlier_rejection_rate": 0.95,  # % of injected outliers caught
        "false_positive_rate": 0.02,     # % valid ranges incorrectly rejected
        "noise_tolerance_5cm": 0.20,     # Max error with ±5cm noise
        "noise_tolerance_10cm": 0.35,    # Max error with ±10cm noise
    },
    "latency": {
        "max_solve_time_ms": 5.0,        # Single trilateration solve
        "max_e2e_latency_ms": 50.0,      # Full pipeline rx → output
    },
    "reliability": {
        "min_solve_success_rate": 0.98,  # % of valid inputs producing output
        "min_quality_correlation": 0.70,  # quality_score correlation with error
    },
}

# Acceptance criteria for detecting improvement vs degradation
SIGNIFICANCE_THRESHOLD = 0.05  # 5% change considered significant


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class TestCaseResult:
    """Result of a single test case execution."""

    name: str
    ground_truth: Tuple[float, float, float]
    estimated_position: Optional[Tuple[float, float, float]]
    error_m: float
    solve_time_ms: float
    success: bool
    quality_score: Optional[float] = None
    anchors_used: int = 3
    residual_rms: Optional[float] = None


@dataclass
class BaselineMetrics:
    """Aggregated baseline metrics for comparison."""

    timestamp: str
    git_commit: str
    git_branch: str
    algorithm_version: str
    test_count: int
    success_count: int
    success_rate: float
    rmse_m: float
    max_error_m: float
    percentile_95_m: float
    mean_error_m: float
    std_error_m: float
    mean_solve_time_ms: float
    test_results: List[Dict[str, Any]]

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BaselineMetrics":
        """Create from dictionary."""
        return cls(**data)


@dataclass
class ComparisonResult:
    """Result of comparing two baselines."""

    metric_name: str
    baseline_value: float
    current_value: float
    delta: float
    delta_percent: float
    verdict: str  # "IMPROVEMENT", "DEGRADATION", "NEUTRAL"
    significant: bool


# =============================================================================
# Utility Functions
# =============================================================================


def get_git_commit() -> str:
    """Get current git commit hash."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return result.stdout.strip()[:8] if result.returncode == 0 else "unknown"
    except Exception:
        return "unknown"


def get_git_branch() -> str:
    """Get current git branch name."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return result.stdout.strip() if result.returncode == 0 else "unknown"
    except Exception:
        return "unknown"


def calculate_distance_3d(
    p1: Tuple[float, float, float], p2: Tuple[float, float, float]
) -> float:
    """Calculate Euclidean distance between two 3D points."""
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2
    )


def calculate_distance_2d(
    p1: Tuple[float, float], p2: Tuple[float, float]
) -> float:
    """Calculate Euclidean distance between two 2D points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def calculate_rmse(errors: List[float]) -> float:
    """Calculate root mean square error."""
    if not errors:
        return float("inf")
    return math.sqrt(sum(e ** 2 for e in errors) / len(errors))


def calculate_percentile(values: List[float], percentile: float) -> float:
    """Calculate percentile value."""
    if not values:
        return float("inf")
    sorted_values = sorted(values)
    index = int(len(sorted_values) * percentile / 100)
    index = min(index, len(sorted_values) - 1)
    return sorted_values[index]


def determine_verdict(
    baseline_value: float,
    current_value: float,
    lower_is_better: bool = True,
) -> Tuple[str, bool]:
    """
    Determine if change is improvement, degradation, or neutral.

    Args:
        baseline_value: Original metric value.
        current_value: New metric value.
        lower_is_better: True for error metrics, False for success rates.

    Returns:
        Tuple of (verdict, is_significant).
    """
    if baseline_value == 0:
        if current_value == 0:
            return "NEUTRAL", False
        return ("DEGRADATION" if lower_is_better else "IMPROVEMENT"), True

    delta_percent = (current_value - baseline_value) / abs(baseline_value) * 100

    if abs(delta_percent) < SIGNIFICANCE_THRESHOLD * 100:
        return "NEUTRAL", False

    if lower_is_better:
        if delta_percent < -SIGNIFICANCE_THRESHOLD * 100:
            return "IMPROVEMENT", True
        elif delta_percent > SIGNIFICANCE_THRESHOLD * 100:
            return "DEGRADATION", True
    else:
        if delta_percent > SIGNIFICANCE_THRESHOLD * 100:
            return "IMPROVEMENT", True
        elif delta_percent < -SIGNIFICANCE_THRESHOLD * 100:
            return "DEGRADATION", True

    return "NEUTRAL", False


# =============================================================================
# Baseline Management
# =============================================================================


class BaselineManager:
    """Manages recording and loading performance baselines."""

    def __init__(self, baseline_path: Optional[Path] = None):
        """
        Initialize baseline manager.

        Args:
            baseline_path: Path to baseline JSON file.
        """
        self.baseline_path = baseline_path or BASELINE_FILE
        self.baseline_path.parent.mkdir(parents=True, exist_ok=True)

    def record_baseline(
        self,
        test_results: List[TestCaseResult],
        algorithm_version: str = "baseline",
    ) -> BaselineMetrics:
        """
        Record performance baseline from test results.

        Args:
            test_results: List of test case results.
            algorithm_version: Version identifier for the algorithm.

        Returns:
            BaselineMetrics object.
        """
        errors = [r.error_m for r in test_results if r.success]
        solve_times = [r.solve_time_ms for r in test_results]

        metrics = BaselineMetrics(
            timestamp=datetime.utcnow().isoformat(),
            git_commit=get_git_commit(),
            git_branch=get_git_branch(),
            algorithm_version=algorithm_version,
            test_count=len(test_results),
            success_count=sum(1 for r in test_results if r.success),
            success_rate=sum(1 for r in test_results if r.success) / len(test_results)
            if test_results
            else 0.0,
            rmse_m=calculate_rmse(errors),
            max_error_m=max(errors) if errors else float("inf"),
            percentile_95_m=calculate_percentile(errors, 95),
            mean_error_m=sum(errors) / len(errors) if errors else float("inf"),
            std_error_m=math.sqrt(
                sum((e - sum(errors) / len(errors)) ** 2 for e in errors) / len(errors)
            )
            if len(errors) > 1
            else 0.0,
            mean_solve_time_ms=sum(solve_times) / len(solve_times)
            if solve_times
            else 0.0,
            test_results=[asdict(r) for r in test_results],
        )

        return metrics

    def save_baseline(self, metrics: BaselineMetrics) -> None:
        """
        Save baseline metrics to file.

        Args:
            metrics: BaselineMetrics to save.
        """
        with open(self.baseline_path, "w") as f:
            json.dump(metrics.to_dict(), f, indent=2)

    def load_baseline(self) -> Optional[BaselineMetrics]:
        """
        Load baseline metrics from file.

        Returns:
            BaselineMetrics or None if file doesn't exist.
        """
        if not self.baseline_path.exists():
            return None

        with open(self.baseline_path, "r") as f:
            data = json.load(f)

        return BaselineMetrics.from_dict(data)

    def compare_baselines(
        self,
        baseline: BaselineMetrics,
        current: BaselineMetrics,
    ) -> List[ComparisonResult]:
        """
        Compare two baselines and return comparison results.

        Args:
            baseline: Original baseline metrics.
            current: Current metrics to compare.

        Returns:
            List of ComparisonResult for each metric.
        """
        comparisons = []

        # Accuracy metrics (lower is better)
        for metric_name, lower_is_better in [
            ("rmse_m", True),
            ("max_error_m", True),
            ("percentile_95_m", True),
            ("mean_error_m", True),
            ("mean_solve_time_ms", True),
            ("success_rate", False),
        ]:
            baseline_value = getattr(baseline, metric_name)
            current_value = getattr(current, metric_name)
            delta = current_value - baseline_value
            delta_percent = (
                (delta / abs(baseline_value) * 100) if baseline_value != 0 else 0.0
            )
            verdict, significant = determine_verdict(
                baseline_value, current_value, lower_is_better
            )

            comparisons.append(
                ComparisonResult(
                    metric_name=metric_name,
                    baseline_value=baseline_value,
                    current_value=current_value,
                    delta=delta,
                    delta_percent=delta_percent,
                    verdict=verdict,
                    significant=significant,
                )
            )

        return comparisons

    def generate_report(
        self, comparisons: List[ComparisonResult]
    ) -> str:
        """
        Generate human-readable comparison report.

        Args:
            comparisons: List of comparison results.

        Returns:
            Formatted report string.
        """
        lines = [
            "=" * 70,
            "PERFORMANCE COMPARISON REPORT",
            "=" * 70,
            "",
        ]

        improvements = []
        degradations = []
        neutral = []

        for c in comparisons:
            sign = "+" if c.delta > 0 else ""
            status = f"[{c.verdict}]" if c.significant else "[NEUTRAL]"
            line = (
                f"  {c.metric_name:25s}: "
                f"{c.baseline_value:10.4f} → {c.current_value:10.4f} "
                f"({sign}{c.delta_percent:6.2f}%) {status}"
            )

            if c.verdict == "IMPROVEMENT":
                improvements.append(line)
            elif c.verdict == "DEGRADATION":
                degradations.append(line)
            else:
                neutral.append(line)

        if improvements:
            lines.append("IMPROVEMENTS:")
            lines.extend(improvements)
            lines.append("")

        if degradations:
            lines.append("DEGRADATIONS:")
            lines.extend(degradations)
            lines.append("")

        if neutral:
            lines.append("UNCHANGED:")
            lines.extend(neutral)
            lines.append("")

        lines.append("=" * 70)

        # Summary
        total = len(comparisons)
        n_improve = len(improvements)
        n_degrade = len(degradations)
        n_neutral = len(neutral)

        if n_degrade > 0:
            overall = "REGRESSION DETECTED"
        elif n_improve > 0:
            overall = "IMPROVEMENT DETECTED"
        else:
            overall = "NO SIGNIFICANT CHANGES"

        lines.append(f"SUMMARY: {overall}")
        lines.append(
            f"  Improvements: {n_improve}/{total}, "
            f"Degradations: {n_degrade}/{total}, "
            f"Neutral: {n_neutral}/{total}"
        )
        lines.append("=" * 70)

        return "\n".join(lines)


# =============================================================================
# Monte Carlo Test Generator
# =============================================================================


class MonteCarloTestGenerator:
    """Generates randomized test cases for statistical comparison."""

    def __init__(
        self,
        anchor_positions: List[Tuple[float, float, float]],
        seed: int = 42,
    ):
        """
        Initialize generator.

        Args:
            anchor_positions: List of anchor (x, y, z) positions.
            seed: Random seed for reproducibility.
        """
        import random

        self.anchors = anchor_positions
        self.random = random.Random(seed)

    def generate_test_cases(
        self,
        count: int = 100,
        x_range: Tuple[float, float] = (-5.0, 15.0),
        y_range: Tuple[float, float] = (-5.0, 15.0),
        z_range: Tuple[float, float] = (0.0, 1.5),
        noise_std_m: float = 0.05,
    ) -> List[Dict[str, Any]]:
        """
        Generate randomized test cases.

        Args:
            count: Number of test cases to generate.
            x_range: (min, max) for X coordinate.
            y_range: (min, max) for Y coordinate.
            z_range: (min, max) for Z coordinate.
            noise_std_m: Standard deviation of distance noise.

        Returns:
            List of test case dictionaries.
        """
        test_cases = []

        for i in range(count):
            # Random ground truth position
            x = self.random.uniform(*x_range)
            y = self.random.uniform(*y_range)
            z = self.random.uniform(*z_range)
            ground_truth = (x, y, z)

            # Calculate true distances
            true_distances = [
                calculate_distance_3d(ground_truth, anchor)
                for anchor in self.anchors
            ]

            # Add noise
            noisy_distances = [
                d + self.random.gauss(0, noise_std_m) for d in true_distances
            ]

            test_cases.append(
                {
                    "name": f"monte_carlo_{i:04d}",
                    "ground_truth": ground_truth,
                    "true_distances_m": true_distances,
                    "noisy_distances_m": noisy_distances,
                    "noise_std_m": noise_std_m,
                }
            )

        return test_cases

    def generate_outlier_cases(
        self,
        count: int = 50,
        outlier_probability: float = 0.2,
        outlier_magnitude_m: float = 5.0,
    ) -> List[Dict[str, Any]]:
        """
        Generate test cases with injected outliers.

        Args:
            count: Number of test cases.
            outlier_probability: Probability of each distance being an outlier.
            outlier_magnitude_m: Magnitude of outlier offset.

        Returns:
            List of test cases with outlier information.
        """
        base_cases = self.generate_test_cases(count)

        for case in base_cases:
            outlier_mask = []
            corrupted_distances = []

            for i, d in enumerate(case["noisy_distances_m"]):
                is_outlier = self.random.random() < outlier_probability
                outlier_mask.append(is_outlier)

                if is_outlier:
                    # Add large offset (positive or negative)
                    offset = self.random.choice([-1, 1]) * outlier_magnitude_m
                    corrupted_distances.append(max(0.1, d + offset))
                else:
                    corrupted_distances.append(d)

            case["corrupted_distances_m"] = corrupted_distances
            case["outlier_mask"] = outlier_mask
            case["has_outliers"] = any(outlier_mask)

        return base_cases
