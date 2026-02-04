"""
Unit tests for Gate Metrics (P1-E).

Tests cover:
- GateMetrics message schema
- Gate geometry calculation
- Perpendicular distance computation
- Crossing detection
- Confidence scoring
- Acceptance criteria verification

Reference: Design Doc Section 8, dev_plan.md P1-E
"""

import pytest
import numpy as np
import time

from bah_core.proto import (
    GateMetrics,
    CrossingEvent,
    create_no_gate_metrics,
    PositionEstimate,
    FixType,
)
from bah_core.domain import (
    GateCalculator,
    GateConfig,
    create_default_gate_calculator,
)
from bah_core.localization import (
    AnchorTracker,
    GNSSQuality,
    GNSSFixType,
)


# =============================================================================
# Test GateMetrics Message Schema
# =============================================================================


class TestGateMetricsSchema:
    """Tests for GateMetrics message schema."""
    
    def test_create_gate_metrics(self):
        """Test creating gate metrics."""
        metrics = GateMetrics(
            tag_id="T1",
            t_solve=time.time(),
            gate_anchor_left_id="A0",
            gate_anchor_right_id="A1",
            d_perp_signed=-2.5,
            s_along=0.3,
            crossing_event=CrossingEvent.NO_CROSSING,
            crossing_confidence=0.0,
        )
        
        assert metrics.tag_id == "T1"
        assert metrics.d_perp_signed == -2.5
        assert metrics.is_left_of_line
        assert not metrics.is_right_of_line
    
    def test_properties(self):
        """Test gate metrics properties."""
        # Left of line
        metrics_left = GateMetrics(
            "T1", time.time(), "A0", "A1", -5.0, 0.5
        )
        assert metrics_left.is_left_of_line
        assert not metrics_left.is_right_of_line
        
        # Right of line
        metrics_right = GateMetrics(
            "T1", time.time(), "A0", "A1", 5.0, 0.5
        )
        assert metrics_right.is_right_of_line
        assert not metrics_right.is_left_of_line
        
        # On line
        metrics_on = GateMetrics(
            "T1", time.time(), "A0", "A1", 0.05, 0.5
        )
        assert metrics_on.is_on_line
        
        # Within bounds
        metrics_within = GateMetrics(
            "T1", time.time(), "A0", "A1", 0.0, 0.5
        )
        assert metrics_within.is_within_gate_bounds
        
        # Outside bounds
        metrics_outside = GateMetrics(
            "T1", time.time(), "A0", "A1", 0.0, 1.5
        )
        assert not metrics_outside.is_within_gate_bounds
    
    def test_crossing_detection(self):
        """Test crossing event detection."""
        metrics_no = GateMetrics(
            "T1", time.time(), "A0", "A1", 0.0, 0.5,
            crossing_event=CrossingEvent.NO_CROSSING
        )
        assert not metrics_no.has_crossing
        
        metrics_left = GateMetrics(
            "T1", time.time(), "A0", "A1", 1.0, 0.5,
            crossing_event=CrossingEvent.CROSSING_LEFT
        )
        assert metrics_left.has_crossing
    
    def test_to_dict(self):
        """Test serialization to dict."""
        metrics = GateMetrics(
            "T1", 1234.5, "A0", "A1", -2.5, 0.3,
            CrossingEvent.CROSSING_LEFT, 0.85
        )
        
        d = metrics.to_dict()
        
        assert d['tag_id'] == "T1"
        assert d['d_perp_signed'] == -2.5
        assert d['crossing_event'] == 'CROSSING_LEFT'
        assert d['crossing_confidence'] == 0.85


# =============================================================================
# Test Gate Geometry Calculation
# =============================================================================


class TestGateGeometry:
    """Tests for gate geometry calculations."""
    
    def create_test_scenario(self):
        """Create test scenario with anchors."""
        # Create anchors at known positions
        anchor_positions = {
            "A0": (0.0, 0.0, 2.0),   # Left
            "A1": (20.0, 0.0, 2.0),  # Right (gate along East axis)
        }
        
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
        
        trackers = {}
        for aid, pos in anchor_positions.items():
            tracker = AnchorTracker(aid)
            tracker.update_gnss(t_now, pos, quality)
            trackers[aid] = tracker
        
        anchor_states = {aid: t.predict(t_now) for aid, t in trackers.items()}
        
        return {
            'anchor_states': anchor_states,
            't_now': t_now,
        }
    
    def create_position_estimate(self, tag_id, t_solve, pos):
        """Create mock position estimate."""
        return PositionEstimate(
            tag_id=tag_id,
            t_solve=t_solve,
            fix_type=FixType.FIX_2D,
            pos_enu=pos,
            quality_score=0.85,
            num_anchors_used=3,
            anchor_ids=["A0", "A1", "A2"],
            residual_m=0.1,
        )
    
    def test_perpendicular_distance_on_line(self):
        """Test perpendicular distance for tag on line."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        # Tag at midpoint of gate (10, 0)
        pos_estimate = self.create_position_estimate(
            "T1", scenario['t_now'], (10.0, 0.0, 0.0)
        )
        
        metrics = calc.compute_metrics(pos_estimate, scenario['anchor_states'])
        
        # Should be on line (d_perp ≈ 0)
        assert abs(metrics.d_perp_signed) < 0.01
        assert 0.4 < metrics.s_along < 0.6  # Around midpoint
    
    def test_perpendicular_distance_left_of_line(self):
        """Test perpendicular distance for tag left of line."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        # Tag at (10, -5) - 5m South of gate
        pos_estimate = self.create_position_estimate(
            "T1", scenario['t_now'], (10.0, -5.0, 0.0)
        )
        
        metrics = calc.compute_metrics(pos_estimate, scenario['anchor_states'])
        
        # Should be left of line (negative, since line is along East)
        assert metrics.d_perp_signed < -4.9
        assert metrics.is_left_of_line
    
    def test_perpendicular_distance_right_of_line(self):
        """Test perpendicular distance for tag right of line."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        # Tag at (10, 5) - 5m North of gate
        pos_estimate = self.create_position_estimate(
            "T1", scenario['t_now'], (10.0, 5.0, 0.0)
        )
        
        metrics = calc.compute_metrics(pos_estimate, scenario['anchor_states'])
        
        # Should be right of line (positive)
        assert metrics.d_perp_signed > 4.9
        assert metrics.is_right_of_line
    
    def test_along_line_coordinate(self):
        """Test along-line coordinate calculation."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        # Test points along line
        test_cases = [
            ((0.0, 0.0, 0.0), 0.0),    # At left anchor
            ((5.0, 0.0, 0.0), 0.25),   # 1/4 way
            ((10.0, 0.0, 0.0), 0.5),   # Midpoint
            ((15.0, 0.0, 0.0), 0.75),  # 3/4 way
            ((20.0, 0.0, 0.0), 1.0),   # At right anchor
        ]
        
        for pos, expected_s in test_cases:
            pos_estimate = self.create_position_estimate("T1", scenario['t_now'], pos)
            metrics = calc.compute_metrics(pos_estimate, scenario['anchor_states'])
            
            assert abs(metrics.s_along - expected_s) < 0.01
    
    def test_outside_gate_bounds(self):
        """Test s_along for positions outside gate segment."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        # Beyond left anchor
        pos_left = self.create_position_estimate("T1", scenario['t_now'], (-5.0, 0.0, 0.0))
        metrics_left = calc.compute_metrics(pos_left, scenario['anchor_states'])
        assert metrics_left.s_along < 0.0
        
        # Beyond right anchor
        pos_right = self.create_position_estimate("T1", scenario['t_now'], (25.0, 0.0, 0.0))
        metrics_right = calc.compute_metrics(pos_right, scenario['anchor_states'])
        assert metrics_right.s_along > 1.0


# =============================================================================
# Test Crossing Detection
# =============================================================================


class TestCrossingDetection:
    """Tests for crossing detection."""
    
    def create_test_scenario(self):
        """Create test scenario."""
        anchor_positions = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (20.0, 0.0, 2.0),
        }
        
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
        
        trackers = {}
        for aid, pos in anchor_positions.items():
            tracker = AnchorTracker(aid)
            tracker.update_gnss(t_now, pos, quality)
            trackers[aid] = tracker
        
        anchor_states = {aid: t.predict(t_now) for aid, t in trackers.items()}
        
        return {'anchor_states': anchor_states, 't_now': t_now}
    
    def create_position_estimate(self, tag_id, t_solve, pos):
        """Create position estimate."""
        return PositionEstimate(
            tag_id=tag_id,
            t_solve=t_solve,
            fix_type=FixType.FIX_2D,
            pos_enu=pos,
            quality_score=0.85,
            num_anchors_used=3,
            anchor_ids=["A0", "A1", "A2"],
            residual_m=0.1,
        )
    
    def test_no_crossing_same_side(self):
        """
        Test no crossing when tag stays on same side.
        
        Acceptance criterion: "Crossing events are detected consistently"
        """
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        t0 = scenario['t_now']
        dt = 0.1
        
        # Tag moves on left side only
        positions = [
            (10.0, -5.0, 0.0),  # Left side
            (10.0, -4.0, 0.0),  # Still left
            (10.0, -3.0, 0.0),  # Still left
        ]
        
        for i, pos in enumerate(positions):
            est = self.create_position_estimate("T1", t0 + i * dt, pos)
            metrics = calc.compute_metrics(est, scenario['anchor_states'])
            
            # All should be left side, no crossing
            assert metrics.is_left_of_line
            assert not metrics.has_crossing
    
    def test_crossing_left_to_right(self):
        """
        Test crossing detection from left to right.
        
        Acceptance criterion: "Crossing events are detected consistently"
        """
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        t0 = scenario['t_now']
        dt = 0.1
        
        # Tag crosses from left (-) to right (+)
        positions = [
            (10.0, -1.0, 0.0),  # Left side
            (10.0, 1.0, 0.0),   # Right side (crossing!)
        ]
        
        # First position
        est0 = self.create_position_estimate("T1", t0, positions[0])
        metrics0 = calc.compute_metrics(est0, scenario['anchor_states'])
        assert metrics0.is_left_of_line
        assert not metrics0.has_crossing
        
        # Second position (crossing)
        est1 = self.create_position_estimate("T1", t0 + dt, positions[1])
        metrics1 = calc.compute_metrics(est1, scenario['anchor_states'])
        assert metrics1.is_right_of_line
        assert metrics1.has_crossing
        assert metrics1.crossing_event == CrossingEvent.CROSSING_LEFT
        assert metrics1.crossing_confidence > 0.0
    
    def test_crossing_right_to_left(self):
        """Test crossing detection from right to left."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        t0 = scenario['t_now']
        dt = 0.1
        
        # Tag crosses from right (+) to left (-)
        positions = [
            (10.0, 1.0, 0.0),   # Right side
            (10.0, -1.0, 0.0),  # Left side (crossing!)
        ]
        
        # First position
        est0 = self.create_position_estimate("T1", t0, positions[0])
        metrics0 = calc.compute_metrics(est0, scenario['anchor_states'])
        assert metrics0.is_right_of_line
        
        # Second position (crossing)
        est1 = self.create_position_estimate("T1", t0 + dt, positions[1])
        metrics1 = calc.compute_metrics(est1, scenario['anchor_states'])
        assert metrics1.is_left_of_line
        assert metrics1.has_crossing
        assert metrics1.crossing_event == CrossingEvent.CROSSING_RIGHT
    
    def test_multiple_crossings(self):
        """Test multiple crossing detections."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        t0 = scenario['t_now']
        dt = 0.1
        
        # Tag crosses multiple times
        positions = [
            (10.0, -1.0, 0.0),  # Left
            (10.0, 1.0, 0.0),   # Right (crossing 1)
            (10.0, -1.0, 0.0),  # Left (crossing 2)
            (10.0, 1.0, 0.0),   # Right (crossing 3)
        ]
        
        crossing_count = 0
        
        for i, pos in enumerate(positions):
            est = self.create_position_estimate("T1", t0 + i * dt, pos)
            metrics = calc.compute_metrics(est, scenario['anchor_states'])
            
            if metrics.has_crossing:
                crossing_count += 1
        
        # Should detect 3 crossings (1→2, 2→3, 3→4)
        assert crossing_count == 3
    
    def test_confidence_scoring(self):
        """Test crossing confidence scoring."""
        scenario = self.create_test_scenario()
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        t0 = scenario['t_now']
        
        # Crossing near line (high confidence)
        est0_close = self.create_position_estimate("T1", t0, (10.0, -0.2, 0.0))
        metrics0 = calc.compute_metrics(est0_close, scenario['anchor_states'])
        
        est1_close = self.create_position_estimate("T1", t0 + 0.1, (10.0, 0.2, 0.0))
        metrics1_close = calc.compute_metrics(est1_close, scenario['anchor_states'])
        
        confidence_close = metrics1_close.crossing_confidence
        
        # Reset for second test
        calc.reset_tag_history("T1")
        
        # Crossing far from line (lower confidence)
        est0_far = self.create_position_estimate("T1", t0, (10.0, -5.0, 0.0))
        calc.compute_metrics(est0_far, scenario['anchor_states'])
        
        est1_far = self.create_position_estimate("T1", t0 + 0.1, (10.0, 5.0, 0.0))
        metrics1_far = calc.compute_metrics(est1_far, scenario['anchor_states'])
        
        confidence_far = metrics1_far.crossing_confidence
        
        # Close crossing should have higher confidence
        assert confidence_close > confidence_far


# =============================================================================
# Test Acceptance Criteria
# =============================================================================


class TestAcceptanceCriteria:
    """Tests for P1-E acceptance criteria."""
    
    def test_stable_distance_with_gnss_jitter(self):
        """
        Test that signed distance is stable despite GNSS jitter.
        
        Acceptance criterion: "Signed distance-to-line is stable on water
        (no obvious GNSS jitter artifacts)"
        """
        # Create anchors with jittery GNSS
        anchor_positions_base = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (20.0, 0.0, 2.0),
        }
        
        t0 = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.5, num_satellites=8)
        
        calc = GateCalculator(GateConfig(anchor_left_id="A0", anchor_right_id="A1"))
        
        # Simulate 10 position estimates with jittery anchors
        d_perp_values = []
        
        for i in range(10):
            t = t0 + i * 0.1
            
            # Add GNSS jitter to anchors (±30cm)
            trackers = {}
            for aid, base_pos in anchor_positions_base.items():
                jitter = np.random.randn(3) * 0.3
                noisy_pos = tuple(p + j for p, j in zip(base_pos, jitter))
                
                tracker = AnchorTracker(aid)
                tracker.update_gnss(t, noisy_pos, quality)
                trackers[aid] = tracker
            
            anchor_states = {aid: tracker.predict(t) for aid, tracker in trackers.items()}
            
            # Tag at fixed position
            tag_pos = (10.0, -5.0, 0.0)
            pos_estimate = PositionEstimate(
                "T1", t, FixType.FIX_2D, tag_pos, 0.85, 3, ["A0", "A1", "A2"], 0.1
            )
            
            metrics = calc.compute_metrics(pos_estimate, anchor_states)
            d_perp_values.append(metrics.d_perp_signed)
        
        # Compute standard deviation of d_perp
        d_perp_std = np.std(d_perp_values)
        
        # With 30cm anchor jitter, d_perp std should be < 50cm
        # (some jitter is expected but not excessive)
        assert d_perp_std < 0.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
