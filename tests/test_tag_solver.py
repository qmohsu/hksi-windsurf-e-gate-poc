"""
Unit tests for Tag Solver (P1-C).

Tests cover:
- TagRangeReport and PositionEstimate schemas
- 2D and 3D multilateration
- Integration with AnchorTracker and AnchorNetworkFusion
- Acceptance criteria verification

Reference: Design Doc Section 7, dev_plan.md P1-C
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
    create_no_fix,
)
from bah_core.localization import (
    TagSolver,
    TagSolverConfig,
    AnchorTracker,
    AnchorNetworkFusion,
    GNSSQuality,
    GNSSFixType,
)
from bah_core.proto import InterAnchorRangeReport, RangeQuality


# =============================================================================
# Test Message Schemas
# =============================================================================


class TestTagRangeReport:
    """Tests for TagRangeReport message."""
    
    def test_create_valid_report(self):
        """Test creating a valid tag range report."""
        t_now = time.time()
        report = TagRangeReport(
            tag_id="T1",
            anchor_id="A0",
            distance_m=5.5,
            t_measurement=t_now,
            t_bah_rx=t_now + 0.01,
            quality=TagRangeQuality.GOOD,
        )
        
        assert report.tag_id == "T1"
        assert report.anchor_id == "A0"
        assert report.is_valid
        assert report.t_range == report.t_bah_rx
    
    def test_negative_distance_raises(self):
        """Test that negative distance raises ValueError."""
        t_now = time.time()
        
        with pytest.raises(ValueError, match="negative"):
            TagRangeReport("T1", "A0", -5.0, t_now, t_now)


class TestPositionEstimate:
    """Tests for PositionEstimate message."""
    
    def test_create_valid_estimate(self):
        """Test creating a valid position estimate."""
        estimate = PositionEstimate(
            tag_id="T1",
            t_solve=time.time(),
            fix_type=FixType.FIX_2D,
            pos_enu=(5.0, 3.0, 0.0),
            quality_score=0.85,
            num_anchors_used=3,
            anchor_ids=["A0", "A1", "A2"],
            residual_m=0.15,
        )
        
        assert estimate.has_valid_fix
        assert estimate.is_2d_fix
        assert estimate.position_2d == (5.0, 3.0)
    
    def test_no_fix_creation(self):
        """Test create_no_fix helper."""
        no_fix = create_no_fix("T1", time.time(), (1.0, 2.0, 0.0))
        
        assert not no_fix.has_valid_fix
        assert no_fix.quality_score == 0.0
        assert no_fix.fix_type == FixType.NO_FIX


# =============================================================================
# Test Tag Solver
# =============================================================================


class TestTagSolver:
    """Tests for tag position solver."""
    
    def create_test_scenario_2d(self) -> Dict:
        """
        Create test scenario with 3 anchors and 1 tag (2D).
        
        Returns:
            Dictionary with anchors, tag position, ranges
        """
        # Anchor positions (equilateral triangle, 20m sides)
        anchor_positions = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (20.0, 0.0, 2.0),
            "A2": (10.0, 17.32, 2.0),
        }
        
        # Tag position (inside triangle)
        tag_position = (10.0, 8.0, 0.0)  # On water surface
        
        # Compute true ranges
        true_ranges = {}
        for aid, anchor_pos in anchor_positions.items():
            dx = tag_position[0] - anchor_pos[0]
            dy = tag_position[1] - anchor_pos[1]
            dz = tag_position[2] - anchor_pos[2]
            true_ranges[aid] = np.sqrt(dx**2 + dy**2 + dz**2)
        
        # Create anchor trackers with GNSS
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
        
        anchor_states = {}
        for aid, pos in anchor_positions.items():
            tracker = AnchorTracker(aid)
            tracker.update_gnss(t_now, pos, quality)
            anchor_states[aid] = tracker.predict(t_now)
        
        # Create tag range reports with small noise
        range_noise = 0.05  # 5cm std dev
        ranges = []
        for aid in ["A0", "A1", "A2"]:
            noisy_range = true_ranges[aid] + np.random.randn() * range_noise
            ranges.append(TagRangeReport(
                tag_id="T1",
                anchor_id=aid,
                distance_m=noisy_range,
                t_measurement=t_now,
                t_bah_rx=t_now + 0.01,
                quality=TagRangeQuality.GOOD,
            ))
        
        return {
            'anchor_positions': anchor_positions,
            'anchor_states': anchor_states,
            'tag_position_true': tag_position,
            'true_ranges': true_ranges,
            'ranges': ranges,
            't_solve': t_now,
        }
    
    def test_2d_solve_accuracy(self):
        """Test 2D multilateration accuracy."""
        scenario = self.create_test_scenario_2d()
        
        solver = TagSolver(TagSolverConfig(prefer_2d=True, water_surface_z_m=0.0))
        estimate = solver.solve(scenario['ranges'], scenario['anchor_states'])
        
        # Should get valid 2D fix
        assert estimate.has_valid_fix
        assert estimate.is_2d_fix
        assert estimate.num_anchors_used == 3
        
        # Check position accuracy
        true_pos = np.array(scenario['tag_position_true'])
        estimated_pos = np.array(estimate.pos_enu)
        
        error_2d = np.linalg.norm(true_pos[:2] - estimated_pos[:2])
        
        # Should be within 20cm (with 5cm range noise)
        assert error_2d < 0.20
        
        # Should have reasonable quality score
        assert estimate.quality_score > 0.6
        
        # Residual should be small
        assert estimate.residual_m < 0.5
    
    def test_3d_solve_accuracy(self):
        """Test 3D multilateration accuracy."""
        scenario = self.create_test_scenario_2d()
        
        # Modify tag to have non-zero altitude
        tag_z = 1.5
        scenario['tag_position_true'] = (10.0, 8.0, tag_z)
        
        # Recompute ranges
        true_ranges = {}
        for aid, anchor_pos in scenario['anchor_positions'].items():
            diff = np.array(scenario['tag_position_true']) - np.array(anchor_pos)
            true_ranges[aid] = np.linalg.norm(diff)
        
        # Update range reports
        t_now = time.time()
        ranges = []
        for aid in ["A0", "A1", "A2"]:
            ranges.append(TagRangeReport(
                "T1", aid, true_ranges[aid] + np.random.randn() * 0.05,
                t_now, t_now + 0.01, TagRangeQuality.GOOD
            ))
        
        # Solve in 3D
        solver = TagSolver(TagSolverConfig(prefer_2d=False))
        estimate = solver.solve(ranges, scenario['anchor_states'])
        
        assert estimate.has_valid_fix
        assert estimate.is_3d_fix
        
        # Check 3D accuracy
        true_pos = np.array(scenario['tag_position_true'])
        estimated_pos = np.array(estimate.pos_enu)
        error_3d = np.linalg.norm(true_pos - estimated_pos)
        
        # 3D with only 3 anchors has more uncertainty, especially in vertical
        assert error_3d < 0.60  # 60cm tolerance for 3D
    
    def test_no_fix_when_missing_range(self):
        """
        Test NO_FIX when not all 3 ranges available.
        
        Acceptance criterion: "If any of the 3 ranges is invalid/missing → output no-fix"
        """
        scenario = self.create_test_scenario_2d()
        
        # Only provide 2 ranges
        solver = TagSolver()
        estimate = solver.solve(
            scenario['ranges'][:2],  # Only 2 ranges
            scenario['anchor_states']
        )
        
        assert not estimate.has_valid_fix
        assert estimate.fix_type == FixType.NO_FIX
        assert estimate.quality_score == 0.0
    
    def test_no_fix_when_invalid_range(self):
        """Test NO_FIX when range fails validity check."""
        scenario = self.create_test_scenario_2d()
        
        # Mark one range as invalid
        scenario['ranges'][0].quality = TagRangeQuality.INVALID
        
        solver = TagSolver()
        estimate = solver.solve(scenario['ranges'], scenario['anchor_states'])
        
        assert not estimate.has_valid_fix
    
    def test_no_fix_when_missing_anchor_state(self):
        """Test NO_FIX when anchor state unavailable."""
        scenario = self.create_test_scenario_2d()
        
        # Remove one anchor state
        anchor_states = {k: v for k, v in scenario['anchor_states'].items() if k != "A2"}
        
        solver = TagSolver()
        estimate = solver.solve(scenario['ranges'], anchor_states)
        
        assert not estimate.has_valid_fix
    
    def test_stable_position_with_moving_anchors(self):
        """
        Test that tag position updates smoothly when anchors move.
        
        Acceptance criterion: "Tag position updates do not show large jumps
        when anchors move smoothly between GNSS updates"
        """
        # Create scenario
        scenario = self.create_test_scenario_2d()
        solver = TagSolver(TagSolverConfig(prefer_2d=True, water_surface_z_m=0.0))
        
        # Solve at t0
        estimate_t0 = solver.solve(scenario['ranges'], scenario['anchor_states'])
        assert estimate_t0.has_valid_fix
        pos_t0 = np.array(estimate_t0.pos_enu)
        
        # Simulate anchor drift (small smooth motion)
        t1 = scenario['t_solve'] + 1.0
        anchor_drift = 0.2  # 20cm drift in 1 second
        
        anchor_states_t1 = {}
        for aid, state in scenario['anchor_states'].items():
            # Create new tracker with drifted position
            tracker = AnchorTracker(aid)
            quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
            
            # Drift anchor position slightly
            drifted_pos = tuple(p + np.random.randn() * anchor_drift for p in state.pos_enu)
            tracker.update_gnss(t1, drifted_pos, quality)
            anchor_states_t1[aid] = tracker.predict(t1)
        
        # Update ranges based on new anchor positions (tag hasn't moved)
        ranges_t1 = []
        for aid in ["A0", "A1", "A2"]:
            anchor_pos = anchor_states_t1[aid].pos_enu
            tag_pos = scenario['tag_position_true']
            diff = np.array(tag_pos) - np.array(anchor_pos)
            new_range = np.linalg.norm(diff) + np.random.randn() * 0.05
            
            ranges_t1.append(TagRangeReport(
                "T1", aid, new_range, t1, t1 + 0.01, TagRangeQuality.GOOD
            ))
        
        # Solve at t1
        estimate_t1 = solver.solve(ranges_t1, anchor_states_t1)
        assert estimate_t1.has_valid_fix
        pos_t1 = np.array(estimate_t1.pos_enu)
        
        # Tag position should not jump significantly
        position_change = np.linalg.norm(pos_t1 - pos_t0)
        
        # With 20cm anchor drift, tag position should move < 50cm
        assert position_change < 0.50
    
    def test_geometry_score_calculation(self):
        """Test geometry score is computed correctly."""
        scenario = self.create_test_scenario_2d()
        
        solver = TagSolver()
        estimate = solver.solve(scenario['ranges'], scenario['anchor_states'])
        
        # Should have geometry score
        assert estimate.geometry_score is not None
        assert 0.0 <= estimate.geometry_score <= 1.0
        
        # With good triangle, should have decent geometry
        assert estimate.geometry_score > 0.5


# =============================================================================
# Integration Tests
# =============================================================================


class TestP1CIntegration:
    """Integration tests for full P1-C pipeline."""
    
    def test_end_to_end_with_anchor_tracker(self):
        """Test complete pipeline: AnchorTracker → TagSolver."""
        # Setup 3 anchor trackers
        anchor_positions = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (15.0, 0.0, 2.0),
            "A2": (7.5, 13.0, 2.0),
        }
        
        trackers = {}
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
        
        for aid, pos in anchor_positions.items():
            tracker = AnchorTracker(aid)
            # Add noise to GNSS
            noisy_pos = tuple(p + np.random.randn() * 0.3 for p in pos)
            tracker.update_gnss(t_now, noisy_pos, quality)
            trackers[aid] = tracker
        
        # Predict anchor states at solve time
        t_solve = t_now + 0.5
        anchor_states = {aid: t.predict(t_solve) for aid, t in trackers.items()}
        
        # Tag position
        tag_pos = (7.5, 6.5, 0.0)
        
        # Create ranges
        ranges = []
        for aid, anchor_pos in anchor_positions.items():
            diff = np.array(tag_pos) - np.array(anchor_pos)
            true_range = np.linalg.norm(diff)
            noisy_range = true_range + np.random.randn() * 0.1
            
            ranges.append(TagRangeReport(
                "T1", aid, noisy_range, t_solve, t_solve + 0.01, TagRangeQuality.GOOD
            ))
        
        # Solve
        solver = TagSolver(TagSolverConfig(prefer_2d=True, water_surface_z_m=0.0))
        estimate = solver.solve(ranges, anchor_states)
        
        # Verify
        assert estimate.has_valid_fix
        assert estimate.is_2d_fix
        
        # Check accuracy (with 30cm GNSS noise + 10cm range noise)
        error = np.linalg.norm(np.array(tag_pos[:2]) - np.array(estimate.position_2d))
        assert error < 0.7  # Within 70cm
    
    def test_integration_with_fusion(self):
        """Test pipeline: AnchorTracker → AnchorNetworkFusion → TagSolver."""
        # Setup anchors with GNSS
        anchor_positions = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (15.0, 0.0, 2.0),
            "A2": (7.5, 13.0, 2.0),
        }
        
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.5, num_satellites=8)
        
        trackers = {}
        for aid, pos in anchor_positions.items():
            tracker = AnchorTracker(aid)
            noisy_pos = tuple(p + np.random.randn() * 0.5 for p in pos)  # 50cm GNSS noise
            tracker.update_gnss(t_now, noisy_pos, quality)
            trackers[aid] = tracker
        
        # Get GNSS states
        gnss_states = {aid: t.predict(t_now) for aid, t in trackers.items()}
        
        # Create inter-anchor ranges (high quality)
        inter_anchor_ranges = []
        anchor_ids = list(anchor_positions.keys())
        for i, aid_i in enumerate(anchor_ids):
            for aid_j in anchor_ids[i+1:]:
                pos_i = anchor_positions[aid_i]
                pos_j = anchor_positions[aid_j]
                dist = np.linalg.norm(np.array(pos_i) - np.array(pos_j))
                noisy_dist = dist + np.random.randn() * 0.05
                
                inter_anchor_ranges.append(InterAnchorRangeReport(
                    aid_i, aid_j, noisy_dist, t_now, t_now, RangeQuality.EXCELLENT
                ))
        
        # Run fusion (P1-B)
        fusion = AnchorNetworkFusion()
        fusion_result = fusion.fuse(gnss_states, inter_anchor_ranges, t_now)
        
        # Use refined anchor positions
        refined_states = fusion_result.refined_anchors
        
        # Tag position and ranges
        tag_pos = (7.5, 6.5, 0.0)
        tag_ranges = []
        for aid in anchor_ids:
            refined_pos = refined_states[aid].pos_enu
            diff = np.array(tag_pos) - np.array(refined_pos)
            true_range = np.linalg.norm(diff)
            noisy_range = true_range + np.random.randn() * 0.08
            
            tag_ranges.append(TagRangeReport(
                "T1", aid, noisy_range, t_now, t_now + 0.01, TagRangeQuality.GOOD
            ))
        
        # Solve tag position
        solver = TagSolver(TagSolverConfig(prefer_2d=True, water_surface_z_m=0.0))
        estimate = solver.solve(tag_ranges, refined_states)
        
        # Verify
        assert estimate.has_valid_fix
        
        # With fusion, should have reasonable accuracy (GNSS noise + range noise)
        error = np.linalg.norm(np.array(tag_pos[:2]) - np.array(estimate.position_2d))
        assert error < 0.7  # Within 70cm (fusion helps but doesn't eliminate all noise)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
