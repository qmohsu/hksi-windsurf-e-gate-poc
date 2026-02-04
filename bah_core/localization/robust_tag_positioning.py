"""
Robust Tag Positioning Pipeline.

Integrates all P1-D components (range gating, kinematic filtering,
plausibility checking) into a unified robust positioning system.

Usage:
    pipeline = RobustTagPositioningPipeline(config)
    
    # Process tag ranges
    estimate = pipeline.process(ranges, anchor_states, t_solve)
    
    if estimate.has_valid_fix:
        print(f"Position: {estimate.pos_enu}")
        print(f"Quality: {estimate.quality_score}")
    elif estimate.is_held:
        print(f"Held position (age: {estimate.hold_age}s)")
    else:
        print("NO_FIX")

Reference: Design Doc Section 7.3 (Robustness Without Redundancy)
"""

from typing import Dict, List, Optional
from dataclasses import dataclass

from bah_core.localization.tag_solver import TagSolver, TagSolverConfig
from bah_core.localization.tag_range_gating import TagRangeGate, TagRangeGatingConfig
from bah_core.localization.tag_kinematic_filter import TagKinematicFilter, TagKinematicFilterConfig
from bah_core.localization.plausibility_checker import PlausibilityChecker, PlausibilityConfig
from bah_core.localization.anchor_state import AnchorState
from bah_core.proto.tag_range import TagRangeReport
from bah_core.proto.position_estimate import PositionEstimate, FixType
from bah_core.metrics import get_metrics


@dataclass
class RobustPositioningConfig:
    """
    Configuration for robust tag positioning pipeline.
    
    Attributes:
        solver_config: TagSolver configuration
        gating_config: TagRangeGate configuration
        filter_config: TagKinematicFilter configuration
        plausibility_config: PlausibilityChecker configuration
        reject_on_plausibility_fail: If True, output NO_FIX; if False, degrade quality
    """
    
    solver_config: TagSolverConfig = None
    gating_config: TagRangeGatingConfig = None
    filter_config: TagKinematicFilterConfig = None
    plausibility_config: PlausibilityConfig = None
    reject_on_plausibility_fail: bool = False  # Degrade quality by default


class RobustTagPositioningPipeline:
    """
    Unified pipeline for robust tag positioning.
    
    Pipeline stages:
    1. Range gating (reject bad ranges)
    2. Multilateration (if 3 valid ranges)
    3. Plausibility checking (innovation vs filter)
    4. Kinematic filtering (smooth + holdover)
    
    Features:
    - Automatic holdover during brief dropouts
    - Outlier rejection via gating
    - Plausibility-based quality degradation
    - Smooth velocity estimation
    """
    
    def __init__(
        self,
        tag_id: str,
        config: Optional[RobustPositioningConfig] = None
    ):
        """
        Initialize robust positioning pipeline.
        
        Args:
            tag_id: Tag ID for this pipeline
            config: Pipeline configuration (uses defaults if None)
        """
        self.tag_id = tag_id
        self.config = config or RobustPositioningConfig()
        self.metrics = get_metrics()
        
        # Initialize components
        self.solver = TagSolver(self.config.solver_config or TagSolverConfig())
        self.gate = TagRangeGate(self.config.gating_config or TagRangeGatingConfig())
        self.filter = TagKinematicFilter(tag_id, self.config.filter_config or TagKinematicFilterConfig())
        self.plausibility = PlausibilityChecker(self.config.plausibility_config or PlausibilityConfig())
        
        # Track last estimate for plausibility checking
        self._last_estimate: Optional[PositionEstimate] = None
    
    def process(
        self,
        ranges: List[TagRangeReport],
        anchor_states: Dict[str, AnchorState],
        t_solve: float
    ) -> PositionEstimate:
        """
        Process tag ranges to produce robust position estimate.
        
        Args:
            ranges: List of tag range reports
            anchor_states: Anchor states at solve time
            t_solve: Solve time
            
        Returns:
            PositionEstimate (FIX, HOLD, or NO_FIX)
            
        Pipeline:
        1. Gate ranges
        2. If 3 valid → solve multilateration
        3. Check plausibility vs filter prediction
        4. Update kinematic filter
        5. Return filtered/held/no-fix estimate
        """
        self.metrics.increment('robust_pipeline_attempts')
        
        # Stage 1: Gate ranges
        valid_ranges = self.gate.check_batch(ranges)
        
        if len(valid_ranges) < 3:
            # Not enough valid ranges → try hold
            held = self.filter.hold(t_solve)
            if held is not None:
                self.metrics.increment('robust_pipeline_hold')
                return held
            else:
                self.metrics.increment('robust_pipeline_no_fix')
                from bah_core.proto.position_estimate import create_no_fix
                return create_no_fix(self.tag_id, t_solve)
        
        # Stage 2: Solve multilateration
        raw_estimate = self.solver.solve(valid_ranges, anchor_states)
        
        if not raw_estimate.has_valid_fix:
            # Solver failed → try hold
            held = self.filter.hold(t_solve)
            if held is not None:
                self.metrics.increment('robust_pipeline_hold')
                return held
            else:
                self.metrics.increment('robust_pipeline_no_fix')
                return raw_estimate
        
        # Stage 3: Plausibility checking
        plausible = True
        
        # Check innovation vs filter prediction
        if self.filter.is_initialized():
            predicted_pos = self.filter.get_predicted_position(t_solve)
            if predicted_pos is not None:
                if not self.plausibility.check_innovation(raw_estimate, predicted_pos):
                    plausible = False
        
        # Check speed/acceleration vs previous
        if self._last_estimate is not None and self._last_estimate.has_valid_fix:
            dt = t_solve - self._last_estimate.t_solve
            if not self.plausibility.check_estimate(raw_estimate, self._last_estimate, dt):
                plausible = False
        
        # Handle implausible estimate
        if not plausible:
            if self.config.reject_on_plausibility_fail:
                # Reject: try hold
                held = self.filter.hold(t_solve)
                if held is not None:
                    self.metrics.increment('robust_pipeline_hold')
                    return held
                else:
                    self.metrics.increment('robust_pipeline_no_fix')
                    from bah_core.proto.position_estimate import create_no_fix
                    return create_no_fix(self.tag_id, t_solve)
            else:
                # Degrade quality
                raw_estimate = self.plausibility.degrade_quality(raw_estimate, "implausible")
        
        # Stage 4: Kinematic filtering
        filtered_estimate = self.filter.update(raw_estimate)
        
        self._last_estimate = filtered_estimate
        self.metrics.increment('robust_pipeline_success')
        
        return filtered_estimate
    
    def reset(self):
        """Reset pipeline state (filter, history)."""
        self.filter.reset()
        self._last_estimate = None
        self.metrics.increment('robust_pipeline_resets')
    
    def get_statistics(self) -> dict:
        """Get pipeline statistics."""
        return {
            'attempts': self.metrics.get_counter('robust_pipeline_attempts'),
            'successes': self.metrics.get_counter('robust_pipeline_success'),
            'holds': self.metrics.get_counter('robust_pipeline_hold'),
            'no_fixes': self.metrics.get_counter('robust_pipeline_no_fix'),
            'resets': self.metrics.get_counter('robust_pipeline_resets'),
            'gate_stats': self.gate.get_statistics(),
            'plausibility_stats': self.plausibility.get_statistics(),
        }


def create_default_pipeline(tag_id: str) -> RobustTagPositioningPipeline:
    """
    Create robust positioning pipeline with default configuration.
    
    Args:
        tag_id: Tag ID
        
    Returns:
        Configured RobustTagPositioningPipeline
    """
    config = RobustPositioningConfig(
        solver_config=TagSolverConfig(prefer_2d=True, water_surface_z_m=0.0),
        gating_config=TagRangeGatingConfig(),
        filter_config=TagKinematicFilterConfig(),
        plausibility_config=PlausibilityConfig(),
        reject_on_plausibility_fail=False,  # Degrade quality, don't reject
    )
    
    return RobustTagPositioningPipeline(tag_id, config)
