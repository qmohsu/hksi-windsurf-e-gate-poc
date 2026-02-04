"""
Tag Position Solver (3-Anchor Multilateration).

Implements time-aligned tag position solving using exactly 3 anchors.
Supports both 2D (water surface) and 3D solves.

Reference: Design Doc Section 7 (Tag Localization Engine)
"""

from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import numpy as np
import time

from bah_core.proto.tag_range import TagRangeReport, TagRangeBatch
from bah_core.proto.position_estimate import PositionEstimate, FixType, create_no_fix
from bah_core.localization.anchor_state import AnchorState, AnchorStateTable
from bah_core.metrics import get_metrics


@dataclass
class TagSolverConfig:
    """
    Configuration for tag position solver.
    
    Attributes:
        prefer_2d: If True, prefer 2D solve on water surface
        water_surface_z_m: Z coordinate of water surface in ENU frame (m)
        max_iterations: Maximum iterations for nonlinear solve
        convergence_tol_m: Convergence threshold (m)
        min_geometry_score: Minimum acceptable geometry score (0-1)
        max_residual_m: Maximum acceptable residual (m)
    """
    
    prefer_2d: bool = True
    water_surface_z_m: float = 0.0
    max_iterations: int = 20
    convergence_tol_m: float = 0.01  # 1cm
    min_geometry_score: float = 0.1  # Very permissive (3 anchors = limited geometry)
    max_residual_m: float = 5.0      # 5m max residual


class TagSolver:
    """
    Solve tag position using 3-anchor multilateration.
    
    Usage:
        solver = TagSolver(config)
        
        # Get anchor states at solve time
        anchor_states = {
            "A0": tracker_a0.predict(t_range),
            "A1": tracker_a1.predict(t_range),
            "A2": tracker_a2.predict(t_range),
        }
        
        # Or use refined states from P1-B fusion
        anchor_states = fusion_result.refined_anchors
        
        # Solve tag position
        ranges = [range_a0, range_a1, range_a2]  # TagRangeReport
        estimate = solver.solve(ranges, anchor_states)
        
        if estimate.has_valid_fix:
            print(f"Tag position: {estimate.pos_enu}")
    """
    
    def __init__(self, config: Optional[TagSolverConfig] = None):
        """
        Initialize tag solver.
        
        Args:
            config: Solver configuration (uses defaults if None)
        """
        self.config = config or TagSolverConfig()
        self.metrics = get_metrics()
    
    def solve(
        self,
        ranges: List[TagRangeReport],
        anchor_states: Dict[str, AnchorState],
    ) -> PositionEstimate:
        """
        Solve tag position from ranges and anchor states.
        
        Args:
            ranges: List of 3 TagRangeReport (one per anchor)
            anchor_states: Dictionary of anchor states at t_range
            
        Returns:
            PositionEstimate with fix type, position, and quality
            
        Notes:
            - If not exactly 3 valid ranges → NO_FIX
            - If anchor states missing/invalid → NO_FIX
            - If solve fails/diverges → NO_FIX
        """
        self.metrics.increment('tag_solve_attempts')
        
        # Validate inputs
        if len(ranges) != 3:
            self.metrics.increment_drop('tag_solve_wrong_num_ranges')
            return self._create_no_fix(ranges, "need exactly 3 ranges")
        
        # Check all ranges are for same tag
        tag_ids = set(r.tag_id for r in ranges)
        if len(tag_ids) != 1:
            self.metrics.increment_drop('tag_solve_mixed_tags')
            return self._create_no_fix(ranges, "mixed tag IDs")
        
        tag_id = ranges[0].tag_id
        t_solve = max(r.t_range for r in ranges)  # Use latest t_range
        
        # Check all ranges are valid
        valid_ranges = [r for r in ranges if r.is_valid]
        if len(valid_ranges) != 3:
            self.metrics.increment_drop('tag_solve_invalid_ranges')
            return create_no_fix(tag_id, t_solve)
        
        # Check anchor states available
        anchor_ids = [r.anchor_id for r in valid_ranges]
        missing_anchors = [aid for aid in anchor_ids if aid not in anchor_states]
        if missing_anchors:
            self.metrics.increment_drop('tag_solve_missing_anchors')
            return create_no_fix(tag_id, t_solve)
        
        # Check anchor states valid
        for aid in anchor_ids:
            if not anchor_states[aid].is_valid:
                self.metrics.increment_drop('tag_solve_invalid_anchor_state')
                return create_no_fix(tag_id, t_solve)
        
        # Compute geometry score
        geometry_score = self._compute_geometry_score(anchor_states, anchor_ids)
        if geometry_score < self.config.min_geometry_score:
            self.metrics.increment_drop('tag_solve_poor_geometry')
            return create_no_fix(tag_id, t_solve)
        
        # Run multilateration
        try:
            if self.config.prefer_2d:
                pos_enu, residual, iterations = self._solve_2d(
                    valid_ranges, anchor_states, anchor_ids
                )
                fix_type = FixType.FIX_2D
            else:
                pos_enu, residual, iterations = self._solve_3d(
                    valid_ranges, anchor_states, anchor_ids
                )
                fix_type = FixType.FIX_3D
            
            # Check residual
            if residual > self.config.max_residual_m:
                self.metrics.increment_drop('tag_solve_high_residual')
                return create_no_fix(tag_id, t_solve)
            
            # Compute quality score
            quality_score = self._compute_quality_score(
                geometry_score, residual, anchor_states, anchor_ids
            )
            
            # Create estimate
            estimate = PositionEstimate(
                tag_id=tag_id,
                t_solve=t_solve,
                fix_type=fix_type,
                pos_enu=tuple(pos_enu),
                quality_score=quality_score,
                num_anchors_used=3,
                anchor_ids=anchor_ids,
                residual_m=residual,
                geometry_score=geometry_score,
            )
            
            self.metrics.increment('tag_solve_success')
            self.metrics.record_histogram('tag_solve_iterations', iterations)
            self.metrics.record_histogram('tag_solve_residual_m', residual)
            
            return estimate
            
        except Exception as e:
            self.metrics.increment_drop('tag_solve_exception')
            return create_no_fix(tag_id, t_solve)
    
    def _solve_2d(
        self,
        ranges: List[TagRangeReport],
        anchor_states: Dict[str, AnchorState],
        anchor_ids: List[str]
    ) -> Tuple[np.ndarray, float, int]:
        """
        Solve 2D position on water surface using 3 anchors.
        
        Returns:
            Tuple of (pos_enu, residual_m, iterations)
        """
        # Extract anchor positions (2D on surface)
        anchor_positions_2d = []
        for aid in anchor_ids:
            pos = anchor_states[aid].pos_enu
            anchor_positions_2d.append((pos[0], pos[1]))  # (E, N)
        
        # Extract ranges
        measured_ranges = [r.distance_m for r in ranges]
        
        # Initial guess: centroid of anchors
        x0 = np.mean([p[0] for p in anchor_positions_2d])
        y0 = np.mean([p[1] for p in anchor_positions_2d])
        x_init = np.array([x0, y0])
        
        # Gauss-Newton iteration
        x = x_init.copy()
        for iteration in range(self.config.max_iterations):
            # Compute residuals and Jacobian
            residuals = np.zeros(3)
            jacobian = np.zeros((3, 2))
            
            for i, (ax, ay) in enumerate(anchor_positions_2d):
                # Computed range from current estimate to anchor
                dx = x[0] - ax
                dy = x[1] - ay
                computed_range = np.sqrt(dx**2 + dy**2)
                
                # Residual: computed - measured
                residuals[i] = computed_range - measured_ranges[i]
                
                # Jacobian: ∂r/∂x, ∂r/∂y
                if computed_range > 1e-6:
                    jacobian[i, 0] = dx / computed_range
                    jacobian[i, 1] = dy / computed_range
            
            # Solve normal equations: J^T J Δx = -J^T r
            JTJ = jacobian.T @ jacobian
            JTr = jacobian.T @ residuals
            
            try:
                delta_x = np.linalg.solve(JTJ + 1e-6 * np.eye(2), -JTr)
            except np.linalg.LinAlgError:
                delta_x = np.linalg.lstsq(JTJ, -JTr, rcond=None)[0]
            
            # Update
            x += delta_x
            
            # Check convergence
            if np.linalg.norm(delta_x) < self.config.convergence_tol_m:
                break
        
        # Final residual (RMS)
        final_residuals = []
        for i, (ax, ay) in enumerate(anchor_positions_2d):
            dx = x[0] - ax
            dy = x[1] - ay
            computed_range = np.sqrt(dx**2 + dy**2)
            final_residuals.append(computed_range - measured_ranges[i])
        
        residual_rms = np.sqrt(np.mean(np.array(final_residuals) ** 2))
        
        # Create 3D position with fixed Z (water surface)
        pos_enu = np.array([x[0], x[1], self.config.water_surface_z_m])
        
        return pos_enu, residual_rms, iteration + 1
    
    def _solve_3d(
        self,
        ranges: List[TagRangeReport],
        anchor_states: Dict[str, AnchorState],
        anchor_ids: List[str]
    ) -> Tuple[np.ndarray, float, int]:
        """
        Solve 3D position using 3 anchors.
        
        Returns:
            Tuple of (pos_enu, residual_m, iterations)
        """
        # Extract anchor positions (3D)
        anchor_positions = []
        for aid in anchor_ids:
            pos = anchor_states[aid].pos_enu
            anchor_positions.append((pos[0], pos[1], pos[2]))
        
        # Extract ranges
        measured_ranges = [r.distance_m for r in ranges]
        
        # Initial guess: centroid of anchors
        x0 = np.mean([p[0] for p in anchor_positions])
        y0 = np.mean([p[1] for p in anchor_positions])
        z0 = np.mean([p[2] for p in anchor_positions])
        x_init = np.array([x0, y0, z0])
        
        # Gauss-Newton iteration
        x = x_init.copy()
        for iteration in range(self.config.max_iterations):
            # Compute residuals and Jacobian
            residuals = np.zeros(3)
            jacobian = np.zeros((3, 3))
            
            for i, (ax, ay, az) in enumerate(anchor_positions):
                # Computed range
                dx = x[0] - ax
                dy = x[1] - ay
                dz = x[2] - az
                computed_range = np.sqrt(dx**2 + dy**2 + dz**2)
                
                # Residual
                residuals[i] = computed_range - measured_ranges[i]
                
                # Jacobian
                if computed_range > 1e-6:
                    jacobian[i, 0] = dx / computed_range
                    jacobian[i, 1] = dy / computed_range
                    jacobian[i, 2] = dz / computed_range
            
            # Solve
            JTJ = jacobian.T @ jacobian
            JTr = jacobian.T @ residuals
            
            try:
                delta_x = np.linalg.solve(JTJ + 1e-6 * np.eye(3), -JTr)
            except np.linalg.LinAlgError:
                delta_x = np.linalg.lstsq(JTJ, -JTr, rcond=None)[0]
            
            # Update
            x += delta_x
            
            # Check convergence
            if np.linalg.norm(delta_x) < self.config.convergence_tol_m:
                break
        
        # Final residual
        final_residuals = []
        for i, (ax, ay, az) in enumerate(anchor_positions):
            dx = x[0] - ax
            dy = x[1] - ay
            dz = x[2] - az
            computed_range = np.sqrt(dx**2 + dy**2 + dz**2)
            final_residuals.append(computed_range - measured_ranges[i])
        
        residual_rms = np.sqrt(np.mean(np.array(final_residuals) ** 2))
        
        return x, residual_rms, iteration + 1
    
    def _compute_geometry_score(
        self,
        anchor_states: Dict[str, AnchorState],
        anchor_ids: List[str]
    ) -> float:
        """
        Compute geometry health score (0-1).
        
        Based on triangle area (larger = better geometry for 3 anchors).
        
        Args:
            anchor_states: Anchor states
            anchor_ids: List of 3 anchor IDs
            
        Returns:
            Geometry score between 0 (degenerate) and 1 (good)
        """
        # Get anchor positions
        positions = [anchor_states[aid].pos_enu[:2] for aid in anchor_ids]  # 2D (E, N)
        
        # Compute triangle area using cross product
        p0 = np.array(positions[0])
        p1 = np.array(positions[1])
        p2 = np.array(positions[2])
        
        # Area = 0.5 * |cross product|
        area = 0.5 * abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - 
                        (p2[0] - p0[0]) * (p1[1] - p0[1]))
        
        # Score based on area (heuristic: good area ~ 100 m² for 20m triangle)
        # Score = 1 - exp(-area / characteristic_area)
        characteristic_area = 50.0  # m²
        score = 1.0 - np.exp(-area / characteristic_area)
        
        return float(np.clip(score, 0.0, 1.0))
    
    def _compute_quality_score(
        self,
        geometry_score: float,
        residual_m: float,
        anchor_states: Dict[str, AnchorState],
        anchor_ids: List[str]
    ) -> float:
        """
        Compute overall quality score (0-1).
        
        Considers:
        - Geometry health
        - Residual magnitude
        - Anchor quality (GNSS/fusion uncertainty)
        
        Returns:
            Quality score between 0 (poor) and 1 (excellent)
        """
        # Component 1: Geometry (30%)
        geo_score = geometry_score
        
        # Component 2: Residual (40%)
        # Lower residual = higher score
        residual_score = np.exp(-residual_m / 0.5)  # 0.5m characteristic scale
        
        # Component 3: Anchor quality (30%)
        anchor_scores = []
        for aid in anchor_ids:
            state = anchor_states[aid]
            # Simple heuristic: degraded anchors reduce quality
            if state.is_degraded:
                anchor_scores.append(0.5)
            else:
                anchor_scores.append(1.0)
        anchor_score = np.mean(anchor_scores)
        
        # Combined score (weighted average)
        quality = 0.3 * geo_score + 0.4 * residual_score + 0.3 * anchor_score
        
        return float(np.clip(quality, 0.0, 1.0))
    
    def _create_no_fix(
        self,
        ranges: List[TagRangeReport],
        reason: str
    ) -> PositionEstimate:
        """Create NO_FIX estimate with last known position."""
        tag_id = ranges[0].tag_id if ranges else "UNKNOWN"
        t_solve = max((r.t_range for r in ranges), default=time.time())
        
        return create_no_fix(tag_id, t_solve)
