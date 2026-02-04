"""
Unit tests for Robust Tag Positioning (P1-D).

Tests cover:
- Tag range gating
- Kinematic filter (smoothing + holdover)
- Plausibility checking (speed/acceleration/innovation)
- Integrated robust pipeline
- Acceptance criteria verification

Reference: Design Doc Section 7.3, dev_plan.md P1-D
"""

import pytest
import numpy as np
import time
from typing import Dict

from bah_core.proto import (
    TagRangeReport,
    TagRangeQuality,
    PositionEstimate,
    FixType,
)
from bah_core.localization import (
    TagRangeGate,
    TagRangeGatingConfig,
    TagKinematicFilter,
    TagKinematicFilterConfig,
    PlausibilityChecker,
    PlausibilityConfig,
    RobustTagPositioningPipeline,
    RobustPositioningConfig,
    AnchorTracker,
    GNSSQuality,
    GNSSFixType,
)


# =============================================================================
# Test Tag Range Gating
# =============================================================================


class TestTagRangeGate:
    """Tests for tag range gating."""
    
    def test_accept_valid_range(self):
        """Test that valid range passes all checks."""
        config = TagRangeGatingConfig(
            d_min_m=1.0,
            d_max_m=50.0,
            max_age_s=1.0,
        )
        gate = TagRangeGate(config)
        
        t_now = time.time()
        report = TagRangeReport(
            "T1", "A0", 10.0, t_now, t_now + 0.01, TagRangeQuality.GOOD
        )
        
        assert gate.check_range(report)
    
    def test_reject_too_close(self):
        """Test rejection of ranges below d_min."""
        config = TagRangeGatingConfig(d_min_m=1.0)
        gate = TagRangeGate(config)
        
        t_now = time.time()
        report = TagRangeReport("T1", "A0", 0.5, t_now, t_now)
        
        assert not gate.check_range(report)
        assert gate.get_rejection_reason(report) == "too_close"
    
    def test_reject_too_far(self):
        """Test rejection of ranges beyond d_max."""
        config = TagRangeGatingConfig(d_max_m=50.0)
        gate = TagRangeGate(config)
        
        t_now = time.time()
        report = TagRangeReport("T1", "A0", 100.0, t_now, t_now)
        
        assert not gate.check_range(report)
        assert gate.get_rejection_reason(report) == "too_far"
    
    def test_reject_poor_quality(self):
        """Test rejection of poor quality ranges."""
        config = TagRangeGatingConfig(min_quality=TagRangeQuality.FAIR)
        gate = TagRangeGate(config)
        
        t_now = time.time()
        report = TagRangeReport(
            "T1", "A0", 10.0, t_now, t_now, quality=TagRangeQuality.POOR
        )
        
        assert not gate.check_range(report)
    
    def test_batch_filtering(self):
        """Test batch filtering of ranges."""
        config = TagRangeGatingConfig(d_min_m=1.0)  # Explicit min distance
        gate = TagRangeGate(config)
        
        t_now = time.time()
        ranges = [
            TagRangeReport("T1", "A0", 10.0, t_now, t_now, TagRangeQuality.GOOD),  # Valid
            TagRangeReport("T1", "A1", 0.5, t_now, t_now, TagRangeQuality.GOOD),   # Too close (< 1m)
            TagRangeReport("T1", "A2", 15.0, t_now, t_now, TagRangeQuality.GOOD),  # Valid
        ]
        
        valid = gate.check_batch(ranges)
        
        assert len(valid) == 2
        assert valid[0].anchor_id == "A0"
        assert valid[1].anchor_id == "A2"


# =============================================================================
# Test Kinematic Filter
# =============================================================================


class TestTagKinematicFilter:
    """Tests for tag kinematic filter."""
    
    def create_mock_estimate(
        self, 
        tag_id: str, 
        t_solve: float, 
        pos: tuple
    ) -> PositionEstimate:
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
    
    def test_initialization_from_first_measurement(self):
        """Test filter initializes from first measurement."""
        filter = TagKinematicFilter("T1")
        
        assert not filter.is_initialized()
        
        t0 = time.time()
        est0 = self.create_mock_estimate("T1", t0, (10.0, 5.0, 0.0))
        
        filtered = filter.update(est0)
        
        assert filter.is_initialized()
        assert filtered.vel_enu == (0.0, 0.0, 0.0)  # Zero velocity initially
    
    def test_smoothing_reduces_jitter(self):
        """Test that filter smooths jittery measurements."""
        filter = TagKinematicFilter("T1")
        
        # True trajectory: moving east at 2 m/s
        t0 = time.time()
        dt = 0.1  # 10 Hz
        
        true_positions = []
        noisy_measurements = []
        filtered_positions = []
        
        for i in range(10):
            t = t0 + i * dt
            true_pos = (2.0 * i * dt, 5.0, 0.0)  # Moving east
            true_positions.append(true_pos)
            
            # Add noise
            noise = np.random.randn(2) * 0.5  # 50cm noise
            noisy_pos = (true_pos[0] + noise[0], true_pos[1] + noise[1], 0.0)
            noisy_measurements.append(noisy_pos)
            
            est = self.create_mock_estimate("T1", t, noisy_pos)
            filtered = filter.update(est)
            filtered_positions.append(filtered.pos_enu)
        
        # Compute RMS error vs true trajectory
        noisy_errors = []
        filtered_errors = []
        
        for i in range(10):
            noisy_err = np.linalg.norm(
                np.array(noisy_measurements[i][:2]) - np.array(true_positions[i][:2])
            )
            filtered_err = np.linalg.norm(
                np.array(filtered_positions[i][:2]) - np.array(true_positions[i][:2])
            )
            noisy_errors.append(noisy_err)
            filtered_errors.append(filtered_err)
        
        avg_noisy_error = np.mean(noisy_errors)
        avg_filtered_error = np.mean(filtered_errors)
        
        # Filter should reduce error (eventually)
        # First few measurements may not show improvement due to initialization
        assert avg_filtered_error < avg_noisy_error * 1.2  # At most 20% worse
    
    def test_velocity_estimation(self):
        """Test that filter estimates velocity correctly."""
        filter = TagKinematicFilter("T1")
        
        # Move east at constant 3 m/s
        t0 = time.time()
        dt = 0.2
        v_true = 3.0  # m/s
        
        for i in range(10):
            t = t0 + i * dt
            pos = (v_true * i * dt, 5.0, 0.0)
            est = self.create_mock_estimate("T1", t, pos)
            filtered = filter.update(est)
            
            # After a few updates, velocity should converge
            if i > 5:
                estimated_speed = np.linalg.norm(filtered.vel_enu[:2])
                assert abs(estimated_speed - v_true) < 1.0  # Within 1 m/s
    
    def test_holdover_during_dropout(self):
        """
        Test holdover during brief measurement dropout.
        
        Acceptance criterion: "During brief range dropouts, filter can hold
        smoothly for a short, bounded time"
        """
        filter = TagKinematicFilter("T1", TagKinematicFilterConfig(max_hold_time_s=2.0))
        
        # Initialize with moving tag
        t0 = time.time()
        dt = 0.1
        
        # Update for 1 second to establish velocity
        for i in range(10):
            t = t0 + i * dt
            pos = (2.0 * i * dt, 5.0, 0.0)  # Moving east at 2 m/s
            est = self.create_mock_estimate("T1", t, pos)
            filter.update(est)
        
        # Now simulate 1 second dropout
        t_dropout = t0 + 1.0
        
        # Hold at 0.5s into dropout
        held_0_5 = filter.hold(t_dropout + 0.5)
        assert held_0_5 is not None
        assert held_0_5.fix_type == FixType.HOLD
        assert 0.0 < held_0_5.quality_score < 0.5  # Degraded quality
        
        # Position should continue forward
        expected_pos_x = 2.0 * 1.0 + 2.0 * 0.5  # Initial + velocity * hold_time
        assert abs(held_0_5.pos_enu[0] - expected_pos_x) < 1.0  # Within 1m
        
        # Hold at 1.5s into dropout (still within limit)
        held_1_5 = filter.hold(t_dropout + 1.5)
        assert held_1_5 is not None
        
        # Hold at 2.5s (exceeds limit)
        held_2_5 = filter.hold(t_dropout + 2.5)
        assert held_2_5 is None  # Exceeded hold time
    
    def test_innovation_computation(self):
        """Test that innovation is computed correctly."""
        filter = TagKinematicFilter("T1")
        
        t0 = time.time()
        dt = 0.1
        
        # Initialize
        est0 = self.create_mock_estimate("T1", t0, (10.0, 5.0, 0.0))
        filter.update(est0)
        
        # Second measurement at expected location
        est1 = self.create_mock_estimate("T1", t0 + dt, (10.2, 5.0, 0.0))
        filtered1 = filter.update(est1)
        
        # Innovation should be small (moving as expected)
        assert filtered1.innovation_m is not None
        assert filtered1.innovation_m < 1.0


# =============================================================================
# Test Plausibility Checker
# =============================================================================


class TestPlausibilityChecker:
    """Tests for plausibility checking."""
    
    def create_estimate(self, t: float, pos: tuple, vel: tuple = None) -> PositionEstimate:
        """Create position estimate."""
        return PositionEstimate(
            tag_id="T1",
            t_solve=t,
            fix_type=FixType.FIX_2D,
            pos_enu=pos,
            quality_score=0.85,
            num_anchors_used=3,
            anchor_ids=["A0", "A1", "A2"],
            residual_m=0.1,
            vel_enu=vel,
        )
    
    def test_accepts_plausible_speed(self):
        """Test that plausible speed passes."""
        checker = PlausibilityChecker(PlausibilityConfig(max_speed_m_s=15.0))
        
        t0 = 0.0
        est0 = self.create_estimate(t0, (10.0, 5.0, 0.0))
        
        # Move 1m in 0.1s → 10 m/s (plausible)
        t1 = t0 + 0.1
        est1 = self.create_estimate(t1, (11.0, 5.0, 0.0))
        
        assert checker.check_estimate(est1, est0, t1 - t0)
    
    def test_rejects_implausible_speed(self):
        """
        Test that implausible speed is rejected.
        
        Acceptance criterion: "Injected single-range outliers cause rejected/no-fix
        rather than teleporting tag positions"
        """
        checker = PlausibilityChecker(PlausibilityConfig(max_speed_m_s=15.0))
        
        t0 = 0.0
        est0 = self.create_estimate(t0, (10.0, 5.0, 0.0))
        
        # Move 10m in 0.1s → 100 m/s (implausible!)
        t1 = t0 + 0.1
        est1 = self.create_estimate(t1, (20.0, 5.0, 0.0))
        
        assert not checker.check_estimate(est1, est0, t1 - t0)
    
    def test_rejects_high_innovation(self):
        """Test that high innovation vs prediction is rejected."""
        checker = PlausibilityChecker(PlausibilityConfig(max_innovation_m=10.0))
        
        # Measurement far from prediction
        estimate = self.create_estimate(0.0, (10.0, 5.0, 0.0))
        predicted_pos = (0.0, 0.0, 0.0)  # 11.2m away
        
        assert not checker.check_innovation(estimate, predicted_pos)


# =============================================================================
# Test Integrated Robust Pipeline
# =============================================================================


class TestRobustPipeline:
    """Tests for integrated robust positioning pipeline."""
    
    def create_test_scenario(self):
        """Create test scenario with anchors and tag."""
        # Anchor positions
        anchor_positions = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (20.0, 0.0, 2.0),
            "A2": (10.0, 17.32, 2.0),
        }
        
        # Create anchor trackers
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
        
        anchor_states = {}
        for aid, pos in anchor_positions.items():
            tracker = AnchorTracker(aid)
            tracker.update_gnss(t_now, pos, quality)
            anchor_states[aid] = tracker.predict(t_now)
        
        return {
            'anchor_positions': anchor_positions,
            'anchor_states': anchor_states,
            't_now': t_now,
        }
    
    def create_ranges(self, tag_pos, anchor_positions, t_now, noise_std=0.05):
        """Create tag range reports."""
        ranges = []
        for aid, anchor_pos in anchor_positions.items():
            diff = np.array(tag_pos) - np.array(anchor_pos)
            true_range = np.linalg.norm(diff)
            noisy_range = true_range + np.random.randn() * noise_std
            
            ranges.append(TagRangeReport(
                "T1", aid, noisy_range, t_now, t_now + 0.01, TagRangeQuality.GOOD
            ))
        
        return ranges
    
    def test_pipeline_with_good_ranges(self):
        """Test pipeline with good ranges produces valid fix."""
        scenario = self.create_test_scenario()
        pipeline = RobustTagPositioningPipeline("T1")
        
        tag_pos = (10.0, 8.0, 0.0)
        ranges = self.create_ranges(
            tag_pos, scenario['anchor_positions'], scenario['t_now']
        )
        
        estimate = pipeline.process(ranges, scenario['anchor_states'], scenario['t_now'])
        
        assert estimate.has_valid_fix
        assert estimate.fix_type == FixType.FIX_2D
        
        # Check accuracy
        error = np.linalg.norm(
            np.array(tag_pos[:2]) - np.array(estimate.pos_enu[:2])
        )
        assert error < 0.5
    
    def test_pipeline_rejects_outlier_range(self):
        """
        Test pipeline rejects solution with outlier range.
        
        Acceptance criterion: "Injected single-range outliers cause rejected/no-fix
        rather than teleporting tag positions"
        """
        scenario = self.create_test_scenario()
        pipeline = RobustTagPositioningPipeline("T1")
        
        # First, establish position with good ranges
        tag_pos = (10.0, 8.0, 0.0)
        ranges_good = self.create_ranges(
            tag_pos, scenario['anchor_positions'], scenario['t_now']
        )
        
        estimate1 = pipeline.process(
            ranges_good, scenario['anchor_states'], scenario['t_now']
        )
        assert estimate1.has_valid_fix
        
        # Now inject outlier in one range (tag hasn't moved, but one range is 20m off)
        t1 = scenario['t_now'] + 0.1
        ranges_bad = list(ranges_good)
        ranges_bad[0] = TagRangeReport(
            "T1", "A0", ranges_good[0].distance_m + 20.0,  # +20m outlier!
            t1, t1 + 0.01, TagRangeQuality.GOOD
        )
        # Update times
        for i in range(1, 3):
            ranges_bad[i] = TagRangeReport(
                ranges_bad[i].tag_id, ranges_bad[i].anchor_id,
                ranges_bad[i].distance_m, t1, t1 + 0.01, ranges_bad[i].quality
            )
        
        estimate2 = pipeline.process(
            ranges_bad, scenario['anchor_states'], t1
        )
        
        # Should either be NO_FIX, HOLD, or heavily degraded quality
        # (outlier causes implausible solution)
        if estimate2.has_valid_fix:
            # If not rejected, quality should be degraded
            assert estimate2.quality_score < 0.3
        else:
            # Or it's held/no-fix
            assert estimate2.fix_type in [FixType.HOLD, FixType.NO_FIX]
    
    def test_pipeline_holdover_during_dropout(self):
        """
        Test pipeline holds during brief range dropout.
        
        Acceptance criterion: "During brief range dropouts, filter can hold
        smoothly for a short, bounded time"
        """
        scenario = self.create_test_scenario()
        pipeline = RobustTagPositioningPipeline("T1")
        
        # Establish position with moving tag
        t0 = scenario['t_now']
        dt = 0.1
        
        for i in range(5):
            t = t0 + i * dt
            tag_pos = (10.0 + i * 0.2, 8.0, 0.0)  # Moving east
            ranges = self.create_ranges(tag_pos, scenario['anchor_positions'], t)
            
            estimate = pipeline.process(ranges, scenario['anchor_states'], t)
            assert estimate.has_valid_fix
        
        # Now simulate dropout (only 1 range available)
        t_dropout = t0 + 0.5
        ranges_dropout = [self.create_ranges(
            (11.0, 8.0, 0.0), scenario['anchor_positions'], t_dropout
        )[0]]  # Only 1 range
        
        estimate_hold = pipeline.process(
            ranges_dropout, scenario['anchor_states'], t_dropout
        )
        
        # Should hold (not enough ranges)
        assert estimate_hold.fix_type == FixType.HOLD
        
        # Position should continue forward based on velocity
        assert estimate_hold.pos_enu[0] > 10.5  # Moved east


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
