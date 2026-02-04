"""
Unit tests for Inter-Anchor Fusion (P1-B).

Tests cover:
- InterAnchorRangeReport message schema
- Range gating (sanity, quality, temporal consistency)
- Anchor network fusion (WLS solver)
- Integration with AnchorTracker
- Acceptance criteria verification

Reference: Design Doc Section 6.5, dev_plan.md P1-B
"""

import pytest
import numpy as np
import time
from typing import Dict

from bah_core.proto import InterAnchorRangeReport, RangeQuality
from bah_core.localization import (
    InterAnchorRangeGate,
    RangeGatingConfig,
    AnchorNetworkFusion,
    FusionConfig,
    AnchorTracker,
    GNSSQuality,
    GNSSFixType,
)


# =============================================================================
# Test InterAnchorRangeReport Message Schema
# =============================================================================


class TestInterAnchorRangeReport:
    """Tests for InterAnchorRangeReport message."""
    
    def test_create_valid_range_report(self):
        """Test creating a valid range report."""
        t_now = time.time()
        report = InterAnchorRangeReport(
            anchor_id_i="A0",
            anchor_id_j="A1",
            distance_m=10.5,
            t_measurement=t_now,
            t_received=t_now + 0.01,
            quality=RangeQuality.GOOD,
        )
        
        assert report.anchor_id_i == "A0"
        assert report.anchor_id_j == "A1"
        assert report.distance_m == 10.5
        assert report.is_valid
    
    def test_negative_distance_raises(self):
        """Test that negative distance raises ValueError."""
        t_now = time.time()
        
        with pytest.raises(ValueError, match="negative"):
            InterAnchorRangeReport(
                anchor_id_i="A0",
                anchor_id_j="A1",
                distance_m=-5.0,
                t_measurement=t_now,
                t_received=t_now,
            )
    
    def test_self_range_raises(self):
        """Test that range to self raises ValueError."""
        t_now = time.time()
        
        with pytest.raises(ValueError, match="self"):
            InterAnchorRangeReport(
                anchor_id_i="A0",
                anchor_id_j="A0",
                distance_m=10.0,
                t_measurement=t_now,
                t_received=t_now,
            )
    
    def test_ordered_pair_canonical(self):
        """Test that ordered pair is canonical regardless of input order."""
        t_now = time.time()
        
        report1 = InterAnchorRangeReport("A0", "A1", 10.0, t_now, t_now)
        report2 = InterAnchorRangeReport("A1", "A0", 10.0, t_now, t_now)
        
        assert report1.get_ordered_pair() == report2.get_ordered_pair()
        assert report1.get_ordered_pair() == ("A0", "A1")
    
    def test_age_calculation(self):
        """Test age_at_processing calculation."""
        t_meas = 100.0
        t_recv = 100.1
        
        report = InterAnchorRangeReport("A0", "A1", 10.0, t_meas, t_recv)
        
        assert abs(report.age_at_processing - 0.1) < 1e-6
    
    def test_variance_by_quality(self):
        """Test that variance scales with quality."""
        t_now = time.time()
        
        excellent = InterAnchorRangeReport("A0", "A1", 10.0, t_now, t_now, 
                                           quality=RangeQuality.EXCELLENT)
        good = InterAnchorRangeReport("A0", "A1", 10.0, t_now, t_now,
                                      quality=RangeQuality.GOOD)
        poor = InterAnchorRangeReport("A0", "A1", 10.0, t_now, t_now,
                                      quality=RangeQuality.POOR)
        
        assert excellent.get_variance() < good.get_variance()
        assert good.get_variance() < poor.get_variance()


# =============================================================================
# Test Range Gating
# =============================================================================


class TestInterAnchorRangeGate:
    """Tests for range gating."""
    
    def test_accept_valid_range(self):
        """Test that valid range passes all checks."""
        config = RangeGatingConfig(
            d_min_m=5.0,
            d_max_m=50.0,
            v_max_m_s=2.0,
            max_age_s=1.0,
        )
        gate = InterAnchorRangeGate(config)
        
        t_now = time.time()
        report = InterAnchorRangeReport(
            "A0", "A1", 15.0, t_now, t_now + 0.01, RangeQuality.GOOD
        )
        
        assert gate.check_range(report)
    
    def test_reject_too_close(self):
        """Test rejection of ranges below d_min."""
        config = RangeGatingConfig(d_min_m=5.0)
        gate = InterAnchorRangeGate(config)
        
        t_now = time.time()
        report = InterAnchorRangeReport("A0", "A1", 2.0, t_now, t_now)
        
        assert not gate.check_range(report)
        assert gate.get_rejection_reason(report) == "too_close"
    
    def test_reject_too_far(self):
        """Test rejection of ranges beyond d_max."""
        config = RangeGatingConfig(d_max_m=50.0)
        gate = InterAnchorRangeGate(config)
        
        t_now = time.time()
        report = InterAnchorRangeReport("A0", "A1", 100.0, t_now, t_now)
        
        assert not gate.check_range(report)
        assert gate.get_rejection_reason(report) == "too_far"
    
    def test_reject_poor_quality(self):
        """Test rejection of poor quality ranges."""
        config = RangeGatingConfig(min_quality=RangeQuality.FAIR)
        gate = InterAnchorRangeGate(config)
        
        t_now = time.time()
        report = InterAnchorRangeReport(
            "A0", "A1", 15.0, t_now, t_now, quality=RangeQuality.POOR
        )
        
        assert not gate.check_range(report)
        assert gate.get_rejection_reason(report) == "poor_quality"
    
    def test_reject_stale(self):
        """Test rejection of stale measurements."""
        config = RangeGatingConfig(max_age_s=1.0)
        gate = InterAnchorRangeGate(config)
        
        t_now = time.time()
        report = InterAnchorRangeReport(
            "A0", "A1", 15.0, t_now - 2.0, t_now  # 2s delay
        )
        
        assert not gate.check_range(report)
        assert gate.get_rejection_reason(report) == "stale"
    
    def test_temporal_consistency_pass(self):
        """Test that small changes pass temporal check."""
        config = RangeGatingConfig(v_max_m_s=2.0, temporal_margin_m=0.5)
        gate = InterAnchorRangeGate(config)
        
        t0 = 100.0
        t1 = 101.0  # 1 second later
        
        # First range: 15m
        report1 = InterAnchorRangeReport("A0", "A1", 15.0, t0, t0)
        assert gate.check_range(report1)
        
        # Second range: 16.5m (1.5m change in 1s, acceptable for v_max=2m/s + margin=0.5m)
        report2 = InterAnchorRangeReport("A0", "A1", 16.5, t1, t1)
        assert gate.check_range(report2)
    
    def test_temporal_consistency_fail(self):
        """Test that large changes fail temporal check."""
        config = RangeGatingConfig(v_max_m_s=2.0, temporal_margin_m=0.5)
        gate = InterAnchorRangeGate(config)
        
        t0 = 100.0
        t1 = 101.0  # 1 second later
        
        # First range: 15m
        report1 = InterAnchorRangeReport("A0", "A1", 15.0, t0, t0)
        assert gate.check_range(report1)
        
        # Second range: 20m (5m change in 1s, too large for v_max=2m/s + margin=0.5m)
        report2 = InterAnchorRangeReport("A0", "A1", 20.0, t1, t1)
        assert not gate.check_range(report2)
        assert gate.get_rejection_reason(report2) == "temporal_inconsistent"


# =============================================================================
# Test Anchor Network Fusion (WLS Solver)
# =============================================================================


class TestAnchorNetworkFusion:
    """Tests for WLS-based anchor network fusion."""
    
    def create_test_scenario(self) -> Dict:
        """
        Create test scenario with 3 anchors in known geometry.
        
        Returns:
            Dictionary with gnss_states, true_positions, ranges
        """
        # True positions (equilateral triangle, 20m sides)
        true_positions = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (20.0, 0.0, 2.0),
            "A2": (10.0, 17.32, 2.0),  # sqrt(3)/2 * 20
        }
        
        # True distances
        d_01 = 20.0
        d_02 = 20.0
        d_12 = 20.0
        
        # Create GNSS states with some noise
        gnss_noise = 1.0  # 1m std dev
        trackers = {}
        gnss_states = {}
        
        for anchor_id, true_pos in true_positions.items():
            tracker = AnchorTracker(anchor_id)
            quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.5, num_satellites=8)
            
            # Add GNSS noise
            noisy_pos = tuple(p + np.random.randn() * gnss_noise for p in true_pos)
            
            t_now = time.time()
            tracker.update_gnss(t_now, noisy_pos, quality)
            gnss_states[anchor_id] = tracker.predict(t_now)
        
        # Create inter-anchor ranges (from true positions, with small noise)
        t_now = time.time()
        range_noise = 0.1  # 10cm std dev
        
        ranges = [
            InterAnchorRangeReport(
                "A0", "A1",
                d_01 + np.random.randn() * range_noise,
                t_now, t_now, RangeQuality.GOOD
            ),
            InterAnchorRangeReport(
                "A0", "A2",
                d_02 + np.random.randn() * range_noise,
                t_now, t_now, RangeQuality.GOOD
            ),
            InterAnchorRangeReport(
                "A1", "A2",
                d_12 + np.random.randn() * range_noise,
                t_now, t_now, RangeQuality.GOOD
            ),
        ]
        
        return {
            'true_positions': true_positions,
            'gnss_states': gnss_states,
            'ranges': ranges,
            't_solve': t_now,
        }
    
    def test_fusion_improves_geometry(self):
        """
        Test that fusion reduces geometry error vs GNSS-only.
        
        This is the key acceptance criterion: inter-anchor fusion should
        reduce relative geometry jitter compared to GNSS-only.
        """
        # Run multiple trials
        trials = 20
        gnss_only_errors = []
        fused_errors = []
        
        for _ in range(trials):
            scenario = self.create_test_scenario()
            
            # Compute GNSS-only geometry error
            gnss_error = self._compute_geometry_error(
                scenario['true_positions'],
                {aid: s.pos_enu for aid, s in scenario['gnss_states'].items()}
            )
            gnss_only_errors.append(gnss_error)
            
            # Run fusion
            fusion = AnchorNetworkFusion()
            result = fusion.fuse(
                scenario['gnss_states'],
                scenario['ranges'],
                scenario['t_solve']
            )
            
            # Compute fused geometry error
            fused_error = self._compute_geometry_error(
                scenario['true_positions'],
                {aid: s.pos_enu for aid, s in result.refined_anchors.items()}
            )
            fused_errors.append(fused_error)
        
        # Average errors
        avg_gnss_error = np.mean(gnss_only_errors)
        avg_fused_error = np.mean(fused_errors)
        
        # Fusion should reduce error
        assert avg_fused_error < avg_gnss_error
        
        # Should see at least 20% improvement
        improvement = (avg_gnss_error - avg_fused_error) / avg_gnss_error
        assert improvement > 0.2
    
    def test_fusion_with_stationary_anchors(self):
        """
        Test that fusion produces stable triangle matching tape measurements.
        
        Acceptance criterion P1-B: "With stationary anchors (lab test),
        refined anchor triangle is stable and matches tape-measured distances"
        """
        scenario = self.create_test_scenario()
        
        fusion = AnchorNetworkFusion()
        result = fusion.fuse(
            scenario['gnss_states'],
            scenario['ranges'],
            scenario['t_solve']
        )
        
        # Check refined distances match measured ranges
        refined_positions = {aid: s.pos_enu for aid, s in result.refined_anchors.items()}
        
        for range_report in scenario['ranges']:
            pos_i = np.array(refined_positions[range_report.anchor_id_i])
            pos_j = np.array(refined_positions[range_report.anchor_id_j])
            
            refined_dist = np.linalg.norm(pos_i - pos_j)
            measured_dist = range_report.distance_m
            
            # Should match within 20cm (reasonable for fusion)
            assert abs(refined_dist - measured_dist) < 0.20
    
    def test_fallback_to_gnss_when_insufficient_ranges(self):
        """Test clean fallback when not enough valid ranges."""
        scenario = self.create_test_scenario()
        
        # Only provide 1 range (need at least 2)
        fusion = AnchorNetworkFusion(FusionConfig(min_ranges_required=2))
        result = fusion.fuse(
            scenario['gnss_states'],
            [scenario['ranges'][0]],  # Only 1 range
            scenario['t_solve']
        )
        
        assert result.fallback_to_gnss
        assert result.num_ranges_used == 1
        assert result.iterations == 0
    
    def test_fallback_when_ranges_fail_gating(self):
        """Test fallback when all ranges fail gating."""
        scenario = self.create_test_scenario()
        
        # Create invalid ranges (too far)
        t_now = time.time()
        bad_ranges = [
            InterAnchorRangeReport("A0", "A1", 200.0, t_now, t_now),  # Too far
        ]
        
        fusion = AnchorNetworkFusion()
        result = fusion.fuse(
            scenario['gnss_states'],
            bad_ranges,
            t_now
        )
        
        assert result.fallback_to_gnss
    
    def test_solver_converges(self):
        """Test that Gauss-Newton solver converges within iteration limit."""
        scenario = self.create_test_scenario()
        
        fusion = AnchorNetworkFusion(FusionConfig(max_iterations=10))
        result = fusion.fuse(
            scenario['gnss_states'],
            scenario['ranges'],
            scenario['t_solve']
        )
        
        # Should converge within iteration limit
        assert result.iterations <= 10
        
        # Residual should be reasonable (weighted norm can vary with noise)
        assert result.residual_norm < 2.0  # Within 2m total weighted residual
    
    def test_fusion_quality_score(self):
        """Test that fusion quality score is computed reasonably."""
        scenario = self.create_test_scenario()
        
        fusion = AnchorNetworkFusion()
        result = fusion.fuse(
            scenario['gnss_states'],
            scenario['ranges'],
            scenario['t_solve']
        )
        
        # Quality should be between 0 and 1
        assert 0.0 <= result.fusion_quality <= 1.0
        
        # With 3 good ranges, should have high quality
        assert result.fusion_quality > 0.6
    
    def _compute_geometry_error(
        self, 
        true_positions: Dict[str, tuple], 
        estimated_positions: Dict[str, tuple]
    ) -> float:
        """
        Compute RMS error in inter-anchor distances (geometry error).
        
        This measures how well the relative geometry is preserved,
        independent of absolute position error.
        """
        anchor_ids = list(true_positions.keys())
        errors_squared = []
        
        for i, aid_i in enumerate(anchor_ids):
            for aid_j in anchor_ids[i+1:]:
                # True distance
                pos_i_true = np.array(true_positions[aid_i])
                pos_j_true = np.array(true_positions[aid_j])
                d_true = np.linalg.norm(pos_i_true - pos_j_true)
                
                # Estimated distance
                pos_i_est = np.array(estimated_positions[aid_i])
                pos_j_est = np.array(estimated_positions[aid_j])
                d_est = np.linalg.norm(pos_i_est - pos_j_est)
                
                # Error in distance
                error = d_est - d_true
                errors_squared.append(error ** 2)
        
        return np.sqrt(np.mean(errors_squared))


# =============================================================================
# Integration Tests
# =============================================================================


class TestP1BIntegration:
    """Integration tests for full P1-B pipeline."""
    
    def test_end_to_end_fusion_pipeline(self):
        """Test complete pipeline: GNSS tracking → range gating → fusion."""
        # Set up 3 anchor trackers
        trackers = {
            "A0": AnchorTracker("A0"),
            "A1": AnchorTracker("A1"),
            "A2": AnchorTracker("A2"),
        }
        
        # True positions
        true_pos = {
            "A0": (0.0, 0.0, 2.0),
            "A1": (15.0, 0.0, 2.0),
            "A2": (7.5, 13.0, 2.0),
        }
        
        # Simulate GNSS updates with noise
        t_now = time.time()
        quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.0, num_satellites=10)
        
        for aid, pos in true_pos.items():
            noisy_pos = tuple(p + np.random.randn() * 0.5 for p in pos)
            trackers[aid].update_gnss(t_now, noisy_pos, quality)
        
        # Get GNSS-tracked states
        gnss_states = {aid: t.predict(t_now) for aid, t in trackers.items()}
        
        # Create inter-anchor ranges
        ranges = [
            InterAnchorRangeReport("A0", "A1", 15.0 + np.random.randn() * 0.05, t_now, t_now),
            InterAnchorRangeReport("A0", "A2", 15.0 + np.random.randn() * 0.05, t_now, t_now),
            InterAnchorRangeReport("A1", "A2", 15.0 + np.random.randn() * 0.05, t_now, t_now),
        ]
        
        # Run fusion with gating
        gate = InterAnchorRangeGate()
        fusion = AnchorNetworkFusion(range_gate=gate)
        
        result = fusion.fuse(gnss_states, ranges, t_now)
        
        # Verify result structure
        assert not result.fallback_to_gnss
        assert result.num_ranges_used == 3
        assert len(result.refined_anchors) == 3
        assert result.fusion_quality > 0.5
        
        # Verify refined positions are reasonable
        for aid, refined_state in result.refined_anchors.items():
            true = np.array(true_pos[aid])
            refined = np.array(refined_state.pos_enu)
            error = np.linalg.norm(refined - true)
            
            # Should be within 1.5m of true position (with GNSS noise ~0.5m)
            assert error < 1.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
