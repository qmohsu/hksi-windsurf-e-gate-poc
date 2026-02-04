"""
Unit tests for Gate Metrics module (Milestone P1-D).

Tests cover:
- Moving start line computation from anchor endpoints
- Signed perpendicular distance (d_perp_signed)
- Along-line progress (s_along)
- Crossing detection and time estimation
- Crossing confidence calculation

Reference: Design Doc Section 7.5 (Gate Metrics)

NOTE: These tests are written BEFORE the GateMetrics implementation.
      They define the expected behavior and will fail until implemented.
"""

import math
import pytest
from typing import Tuple, Optional
from dataclasses import dataclass


# =============================================================================
# Placeholder Classes (To be replaced with actual imports)
# =============================================================================


@dataclass
class GateConfig:
    """Configuration for gate endpoints."""

    left_anchor_id: str
    right_anchor_id: str
    positive_direction: str = "north"  # Which side is "post-start"


@dataclass
class GateMetrics:
    """Gate metrics output structure."""

    tag_id: str
    t_bah: float                    # Solve time
    gate_left_pos: Tuple[float, float, float]   # Left endpoint position
    gate_right_pos: Tuple[float, float, float]  # Right endpoint position
    d_perp_signed: float            # Signed perpendicular distance to line
    s_along: float                  # Along-line progress (0-1)
    crossing_detected: bool
    crossing_time_est: Optional[float]
    crossing_confidence: float


class GateMetricsCalculatorStub:
    """
    Stub implementation for gate metrics calculation.

    Replace with actual implementation when available.
    """

    def __init__(self, config: GateConfig):
        """
        Initialize gate metrics calculator.

        Args:
            config: Gate configuration with endpoint anchor IDs.
        """
        self.config = config
        self._prev_d_perp: Optional[float] = None
        self._prev_time: Optional[float] = None

    def calculate(
        self,
        tag_id: str,
        tag_pos: Tuple[float, float, float],
        t_bah: float,
        left_pos: Tuple[float, float, float],
        right_pos: Tuple[float, float, float],
        position_uncertainty: float = 0.5,
    ) -> GateMetrics:
        """
        Calculate gate metrics for a tag position.

        Args:
            tag_id: Tag identifier.
            tag_pos: Tag position (E, N, U) meters.
            t_bah: BAH solve time.
            left_pos: Left endpoint position.
            right_pos: Right endpoint position.
            position_uncertainty: Tag position uncertainty (meters).

        Returns:
            GateMetrics object.
        """
        # Calculate line vector (left to right)
        line_vec = (
            right_pos[0] - left_pos[0],
            right_pos[1] - left_pos[1],
        )
        line_length = math.sqrt(line_vec[0] ** 2 + line_vec[1] ** 2)

        if line_length < 0.001:
            # Degenerate line
            return GateMetrics(
                tag_id=tag_id,
                t_bah=t_bah,
                gate_left_pos=left_pos,
                gate_right_pos=right_pos,
                d_perp_signed=float("inf"),
                s_along=0.5,
                crossing_detected=False,
                crossing_time_est=None,
                crossing_confidence=0.0,
            )

        # Normalize line vector
        line_unit = (line_vec[0] / line_length, line_vec[1] / line_length)

        # Vector from left endpoint to tag
        tag_vec = (
            tag_pos[0] - left_pos[0],
            tag_pos[1] - left_pos[1],
        )

        # Along-line progress (dot product)
        s_along_raw = (tag_vec[0] * line_unit[0] + tag_vec[1] * line_unit[1]) / line_length
        s_along = max(0.0, min(1.0, s_along_raw))

        # Perpendicular distance (cross product magnitude)
        # Positive if tag is "left" of line (north for E-W line)
        d_perp_signed = tag_vec[0] * (-line_unit[1]) + tag_vec[1] * line_unit[0]

        # Crossing detection
        crossing_detected = False
        crossing_time_est = None

        if self._prev_d_perp is not None and self._prev_time is not None:
            # Check for sign change
            if self._prev_d_perp * d_perp_signed < 0:
                crossing_detected = True
                # Linear interpolation for crossing time
                dt = t_bah - self._prev_time
                if abs(d_perp_signed - self._prev_d_perp) > 0.001:
                    ratio = abs(self._prev_d_perp) / abs(d_perp_signed - self._prev_d_perp)
                    crossing_time_est = self._prev_time + ratio * dt

        # Update previous values
        self._prev_d_perp = d_perp_signed
        self._prev_time = t_bah

        # Confidence based on position uncertainty and distance
        endpoint_uncertainty = 0.5  # Assume anchor uncertainty
        combined_uncertainty = math.sqrt(position_uncertainty ** 2 + endpoint_uncertainty ** 2)
        crossing_confidence = max(0.0, min(1.0, 1.0 - combined_uncertainty / 2.0))

        return GateMetrics(
            tag_id=tag_id,
            t_bah=t_bah,
            gate_left_pos=left_pos,
            gate_right_pos=right_pos,
            d_perp_signed=d_perp_signed,
            s_along=s_along,
            crossing_detected=crossing_detected,
            crossing_time_est=crossing_time_est,
            crossing_confidence=crossing_confidence,
        )


# =============================================================================
# Test Fixtures
# =============================================================================


@pytest.fixture
def gate_config() -> GateConfig:
    """Standard gate configuration."""
    return GateConfig(
        left_anchor_id="A0",
        right_anchor_id="A1",
        positive_direction="north",
    )


@pytest.fixture
def gate_calculator(gate_config: GateConfig) -> GateMetricsCalculatorStub:
    """Gate metrics calculator instance."""
    return GateMetricsCalculatorStub(gate_config)


@pytest.fixture
def horizontal_gate() -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Horizontal gate (E-W) at y=0, from x=0 to x=10."""
    left = (0.0, 0.0, 2.0)
    right = (10.0, 0.0, 2.0)
    return (left, right)


@pytest.fixture
def diagonal_gate() -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Diagonal gate from (0,0) to (10,10)."""
    left = (0.0, 0.0, 2.0)
    right = (10.0, 10.0, 2.0)
    return (left, right)


# =============================================================================
# Perpendicular Distance Tests
# =============================================================================


class TestPerpendicularDistance:
    """Tests for d_perp_signed calculation."""

    def test_d_perp_on_line_is_zero(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test that point on line has zero perpendicular distance."""
        left, right = horizontal_gate
        tag_pos = (5.0, 0.0, 0.0)  # On the line

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert abs(metrics.d_perp_signed) < 0.01, (
            f"d_perp should be ~0 for point on line, got {metrics.d_perp_signed}"
        )

    def test_d_perp_north_is_positive(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test that point north of E-W line has positive d_perp."""
        left, right = horizontal_gate
        tag_pos = (5.0, 5.0, 0.0)  # 5m north of line

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert metrics.d_perp_signed > 0, (
            f"d_perp should be positive (north), got {metrics.d_perp_signed}"
        )
        assert abs(metrics.d_perp_signed - 5.0) < 0.1, (
            f"d_perp should be ~5m, got {metrics.d_perp_signed}"
        )

    def test_d_perp_south_is_negative(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test that point south of E-W line has negative d_perp."""
        left, right = horizontal_gate
        tag_pos = (5.0, -3.0, 0.0)  # 3m south of line

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert metrics.d_perp_signed < 0, (
            f"d_perp should be negative (south), got {metrics.d_perp_signed}"
        )
        assert abs(metrics.d_perp_signed + 3.0) < 0.1, (
            f"d_perp should be ~-3m, got {metrics.d_perp_signed}"
        )

    def test_d_perp_diagonal_gate(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        diagonal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test perpendicular distance for diagonal gate."""
        left, right = diagonal_gate
        # Point at (0, 10) should be perpendicular distance of ~7.07m from line
        tag_pos = (0.0, 10.0, 0.0)

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        expected_d_perp = 10.0 / math.sqrt(2)  # ~7.07m
        assert abs(abs(metrics.d_perp_signed) - expected_d_perp) < 0.1, (
            f"d_perp should be ~{expected_d_perp:.2f}m, got {metrics.d_perp_signed}"
        )


# =============================================================================
# Along-Line Progress Tests
# =============================================================================


class TestAlongLineProgress:
    """Tests for s_along calculation."""

    def test_s_along_at_left_is_zero(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test s_along is 0 at left endpoint."""
        left, right = horizontal_gate
        tag_pos = (0.0, 0.0, 0.0)

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert abs(metrics.s_along) < 0.01, f"s_along should be ~0, got {metrics.s_along}"

    def test_s_along_at_right_is_one(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test s_along is 1 at right endpoint."""
        left, right = horizontal_gate
        tag_pos = (10.0, 0.0, 0.0)

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert abs(metrics.s_along - 1.0) < 0.01, f"s_along should be ~1, got {metrics.s_along}"

    def test_s_along_at_midpoint(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test s_along is 0.5 at midpoint."""
        left, right = horizontal_gate
        tag_pos = (5.0, 3.0, 0.0)  # Midpoint (perpendicular offset doesn't affect s_along)

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert abs(metrics.s_along - 0.5) < 0.05, f"s_along should be ~0.5, got {metrics.s_along}"

    def test_s_along_clipped_beyond_endpoints(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test s_along is clipped to [0, 1] beyond endpoints."""
        left, right = horizontal_gate

        # Beyond left
        metrics_left = gate_calculator.calculate(
            tag_id="T1", tag_pos=(-5.0, 0.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )
        assert metrics_left.s_along == 0.0, "s_along should be clipped to 0"

        # Beyond right
        gate_calculator._prev_d_perp = None  # Reset for independent test
        metrics_right = gate_calculator.calculate(
            tag_id="T1", tag_pos=(15.0, 0.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )
        assert metrics_right.s_along == 1.0, "s_along should be clipped to 1"


# =============================================================================
# Crossing Detection Tests
# =============================================================================


class TestCrossingDetection:
    """Tests for line crossing detection."""

    def test_no_crossing_when_same_side(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test no crossing detected when staying on same side."""
        left, right = horizontal_gate

        # First position: north of line
        gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        # Second position: still north
        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 3.0, 0.0), t_bah=1.0,
            left_pos=left, right_pos=right,
        )

        assert not metrics.crossing_detected, "No crossing should be detected"

    def test_crossing_detected_north_to_south(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test crossing detected when moving north to south."""
        left, right = horizontal_gate

        # First position: north of line
        gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        # Second position: south of line (crossed!)
        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, -1.0, 0.0), t_bah=1.0,
            left_pos=left, right_pos=right,
        )

        assert metrics.crossing_detected, "Crossing should be detected"

    def test_crossing_detected_south_to_north(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test crossing detected when moving south to north."""
        left, right = horizontal_gate

        # First position: south of line
        gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, -2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        # Second position: north of line (crossed!)
        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 1.0, 0.0), t_bah=1.0,
            left_pos=left, right_pos=right,
        )

        assert metrics.crossing_detected, "Crossing should be detected"

    def test_crossing_time_interpolation(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test crossing time is interpolated correctly."""
        left, right = horizontal_gate

        # First position: 2m north of line at t=0
        gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        # Second position: 2m south of line at t=1 (crossed at t≈0.5)
        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, -2.0, 0.0), t_bah=1.0,
            left_pos=left, right_pos=right,
        )

        assert metrics.crossing_time_est is not None, "Crossing time should be estimated"
        assert abs(metrics.crossing_time_est - 0.5) < 0.1, (
            f"Crossing time should be ~0.5, got {metrics.crossing_time_est}"
        )

    def test_crossing_time_asymmetric(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test crossing time with asymmetric distances."""
        left, right = horizontal_gate

        # First position: 1m north at t=0
        gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 1.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        # Second position: 3m south at t=1 (crossed at t≈0.25)
        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, -3.0, 0.0), t_bah=1.0,
            left_pos=left, right_pos=right,
        )

        expected_crossing_time = 0.0 + (1.0 - 0.0) * (1.0 / 4.0)  # 0.25
        assert metrics.crossing_time_est is not None
        assert abs(metrics.crossing_time_est - expected_crossing_time) < 0.1


# =============================================================================
# Confidence Tests
# =============================================================================


class TestCrossingConfidence:
    """Tests for crossing confidence calculation."""

    def test_confidence_higher_with_lower_uncertainty(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test confidence is higher when position uncertainty is low."""
        left, right = horizontal_gate

        # Low uncertainty
        metrics_low = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
            position_uncertainty=0.1,
        )

        # High uncertainty
        gate_calculator._prev_d_perp = None  # Reset
        metrics_high = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
            position_uncertainty=2.0,
        )

        assert metrics_low.crossing_confidence > metrics_high.crossing_confidence, (
            "Lower uncertainty should give higher confidence"
        )

    def test_confidence_bounded_zero_one(
        self,
        gate_calculator: GateMetricsCalculatorStub,
        horizontal_gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    ):
        """Test confidence is bounded to [0, 1]."""
        left, right = horizontal_gate

        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=(5.0, 2.0, 0.0), t_bah=0.0,
            left_pos=left, right_pos=right,
        )

        assert 0.0 <= metrics.crossing_confidence <= 1.0, (
            f"Confidence should be in [0, 1], got {metrics.crossing_confidence}"
        )


# =============================================================================
# Moving Gate Tests
# =============================================================================


class TestMovingGate:
    """Tests for time-varying gate endpoints (buoy drift)."""

    def test_gate_position_changes_over_time(
        self,
        gate_calculator: GateMetricsCalculatorStub,
    ):
        """Test that moving gate endpoints affect metrics."""
        # Gate at t=0
        left_t0 = (0.0, 0.0, 2.0)
        right_t0 = (10.0, 0.0, 2.0)

        # Gate at t=1 (drifted 1m north)
        left_t1 = (0.0, 1.0, 2.0)
        right_t1 = (10.0, 1.0, 2.0)

        tag_pos = (5.0, 0.5, 0.0)  # Fixed tag position

        # Metrics at t=0
        metrics_t0 = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left_t0, right_pos=right_t0,
        )

        # Metrics at t=1 (gate moved, tag didn't)
        gate_calculator._prev_d_perp = None  # Reset to avoid crossing detection
        metrics_t1 = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=1.0,
            left_pos=left_t1, right_pos=right_t1,
        )

        # d_perp should change because gate moved
        assert metrics_t0.d_perp_signed != metrics_t1.d_perp_signed, (
            "d_perp should change when gate moves"
        )

    def test_stationary_tag_crosses_due_to_gate_movement(
        self,
        gate_calculator: GateMetricsCalculatorStub,
    ):
        """Test that stationary tag can 'cross' if gate moves past it."""
        tag_pos = (5.0, 0.0, 0.0)  # Tag stays fixed

        # Gate south of tag at t=0
        left_t0 = (0.0, -2.0, 2.0)
        right_t0 = (10.0, -2.0, 2.0)

        # Gate north of tag at t=1 (moved 4m north)
        left_t1 = (0.0, 2.0, 2.0)
        right_t1 = (10.0, 2.0, 2.0)

        # First measurement
        gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=0.0,
            left_pos=left_t0, right_pos=right_t0,
        )

        # Second measurement
        metrics = gate_calculator.calculate(
            tag_id="T1", tag_pos=tag_pos, t_bah=1.0,
            left_pos=left_t1, right_pos=right_t1,
        )

        # Crossing detected even though tag didn't move
        assert metrics.crossing_detected, (
            "Crossing should be detected when gate moves past stationary tag"
        )
