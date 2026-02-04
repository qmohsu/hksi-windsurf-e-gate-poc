"""
Anchor Network Fusion: GNSS + Inter-Anchor UWB Ranges.

Implements two-stage anchor position refinement:
1. Stage 1: GNSS tracking per anchor (AnchorTracker - already done in P1-A)
2. Stage 2: Geometry refinement using inter-anchor UWB ranges (this module)

Uses weighted least squares (WLS) with Gauss-Newton iterations to solve:

    J(p) = Σ_i ||p_i - p_i^pred||²_{Σ_i^{-1}}  +  Σ_{i<j} (||p_i - p_j|| - d_ij)² / σ²_ij

where:
- p_i^pred: GNSS-tracked anchor positions (priors from AnchorTracker)
- Σ_i: covariance from AnchorTracker
- d_ij: inter-anchor UWB range measurements
- σ²_ij: range measurement variance

Reference: Design Doc Section 6.5.2 (Two-stage fusion architecture)
"""

from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import numpy as np
import time

from bah_core.localization.anchor_state import AnchorState, AnchorStateTable
from bah_core.proto.inter_anchor_range import InterAnchorRangeReport, InterAnchorRangeBatch
from bah_core.localization.range_gating import InterAnchorRangeGate
from bah_core.metrics import get_metrics


@dataclass
class FusionConfig:
    """
    Configuration for anchor network fusion.
    
    Attributes:
        max_iterations: Maximum Gauss-Newton iterations
        convergence_tol_m: Convergence threshold (m)
        min_ranges_required: Minimum valid inter-anchor ranges needed
        gnss_weight_scale: Scale factor for GNSS prior weight (1.0 = use covariance as-is)
        fallback_to_gnss: If True, return GNSS-only if fusion fails
    """
    
    max_iterations: int = 5
    convergence_tol_m: float = 0.01  # 1cm convergence
    min_ranges_required: int = 2     # Need at least 2 ranges for useful fusion
    gnss_weight_scale: float = 1.0   # Can tune GNSS vs UWB relative weight
    fallback_to_gnss: bool = True


@dataclass
class FusionResult:
    """
    Result of anchor network fusion.
    
    Attributes:
        t_solve: Time at which fusion was computed
        refined_anchors: Dictionary of refined anchor states (anchor_id -> AnchorState)
        iterations: Number of Gauss-Newton iterations used
        residual_norm: Final residual norm (convergence indicator)
        num_ranges_used: Number of inter-anchor ranges used
        fusion_quality: Scalar quality indicator (0-1, higher is better)
        fallback_to_gnss: True if fusion failed and GNSS-only was used
    """
    
    t_solve: float
    refined_anchors: Dict[str, AnchorState]
    iterations: int
    residual_norm: float
    num_ranges_used: int
    fusion_quality: float
    fallback_to_gnss: bool = False


class AnchorNetworkFusion:
    """
    Refine anchor positions using GNSS priors + inter-anchor UWB ranges.
    
    Usage:
        fusion = AnchorNetworkFusion(config)
        
        # Get GNSS-tracked states
        gnss_states = {
            "A0": tracker_a0.predict(t),
            "A1": tracker_a1.predict(t),
            "A2": tracker_a2.predict(t),
        }
        
        # Get inter-anchor ranges
        ranges = [range_a0_a1, range_a0_a2, range_a1_a2]
        
        # Fuse
        result = fusion.fuse(gnss_states, ranges, t)
        
        # Use refined positions
        refined_pos = result.refined_anchors["A0"].pos_enu
    """
    
    def __init__(
        self, 
        config: Optional[FusionConfig] = None,
        range_gate: Optional[InterAnchorRangeGate] = None
    ):
        """
        Initialize fusion module.
        
        Args:
            config: Fusion configuration (uses defaults if None)
            range_gate: Range gating instance (creates default if None)
        """
        self.config = config or FusionConfig()
        self.range_gate = range_gate or InterAnchorRangeGate()
        self.metrics = get_metrics()
    
    def fuse(
        self,
        gnss_states: Dict[str, AnchorState],
        range_reports: List[InterAnchorRangeReport],
        t_solve: float
    ) -> FusionResult:
        """
        Fuse GNSS priors with inter-anchor ranges to refine anchor positions.
        
        Args:
            gnss_states: Dictionary of GNSS-tracked anchor states (from AnchorTracker)
            range_reports: List of inter-anchor range measurements
            t_solve: Time at which to compute refined positions
            
        Returns:
            FusionResult with refined anchor positions
            
        Notes:
            - If fusion fails or insufficient ranges, falls back to GNSS-only
            - All positions are in ENU frame
            - Covariances are updated to reflect fusion uncertainty
        """
        self.metrics.increment('anchor_fusion_attempts')
        
        # Gate ranges
        valid_ranges = [r for r in range_reports if self.range_gate.check_range(r)]
        
        # Check if we have enough valid ranges
        if len(valid_ranges) < self.config.min_ranges_required:
            self.metrics.increment('anchor_fusion_fallback_insufficient_ranges')
            return self._fallback_to_gnss(gnss_states, t_solve, num_ranges=len(valid_ranges))
        
        # Check if we have GNSS states for all anchors in ranges
        anchor_ids = set()
        for r in valid_ranges:
            anchor_ids.add(r.anchor_id_i)
            anchor_ids.add(r.anchor_id_j)
        
        missing_ids = [aid for aid in anchor_ids if aid not in gnss_states]
        if missing_ids:
            self.metrics.increment('anchor_fusion_fallback_missing_gnss')
            return self._fallback_to_gnss(gnss_states, t_solve, num_ranges=len(valid_ranges))
        
        # Run WLS optimization
        try:
            refined_positions, iterations, residual = self._solve_wls(
                gnss_states,
                valid_ranges,
                anchor_ids
            )
            
            # Create refined anchor states
            refined_anchors = {}
            for anchor_id in anchor_ids:
                gnss_state = gnss_states[anchor_id]
                refined_pos = refined_positions[anchor_id]
                
                # Create new state with refined position
                # Keep velocity and other attributes from GNSS state
                refined_anchors[anchor_id] = AnchorState(
                    anchor_id=anchor_id,
                    time=t_solve,
                    pos_enu=refined_pos,
                    vel_enu=gnss_state.vel_enu,
                    pos_cov=self._estimate_fused_covariance(gnss_state, len(valid_ranges)),
                    last_gnss_time=gnss_state.last_gnss_time,
                    last_gnss_quality=gnss_state.last_gnss_quality,
                    is_valid=True,
                    is_degraded=False,  # Fusion improves quality
                )
            
            # Compute fusion quality (0-1 scale)
            fusion_quality = self._compute_fusion_quality(
                gnss_states, 
                refined_anchors, 
                valid_ranges,
                residual
            )
            
            self.metrics.increment('anchor_fusion_success')
            self.metrics.record_histogram('anchor_fusion_iterations', iterations)
            self.metrics.record_histogram('anchor_fusion_residual_m', residual)
            
            return FusionResult(
                t_solve=t_solve,
                refined_anchors=refined_anchors,
                iterations=iterations,
                residual_norm=residual,
                num_ranges_used=len(valid_ranges),
                fusion_quality=fusion_quality,
                fallback_to_gnss=False
            )
            
        except Exception as e:
            self.metrics.increment('anchor_fusion_solve_failed')
            self.metrics.increment_drop('fusion_solver_error')
            
            if self.config.fallback_to_gnss:
                return self._fallback_to_gnss(gnss_states, t_solve, num_ranges=len(valid_ranges))
            else:
                raise
    
    def _solve_wls(
        self,
        gnss_states: Dict[str, AnchorState],
        ranges: List[InterAnchorRangeReport],
        anchor_ids: set
    ) -> Tuple[Dict[str, Tuple[float, float, float]], int, float]:
        """
        Solve weighted least squares problem using Gauss-Newton.
        
        Args:
            gnss_states: GNSS prior states
            ranges: Valid inter-anchor ranges
            anchor_ids: Set of anchor IDs to optimize
            
        Returns:
            Tuple of (refined_positions_dict, iterations, final_residual)
        """
        # Create anchor ID to index mapping
        id_to_idx = {aid: i for i, aid in enumerate(sorted(anchor_ids))}
        n_anchors = len(anchor_ids)
        
        # Initialize positions from GNSS (3D: E, N, U)
        x = np.zeros(n_anchors * 3)
        for aid, idx in id_to_idx.items():
            pos = gnss_states[aid].pos_enu
            x[idx*3:(idx+1)*3] = pos
        
        # Build prior weights from GNSS covariance
        W_gnss = self._build_gnss_weights(gnss_states, id_to_idx)
        
        # Build range weights
        W_range = np.array([1.0 / r.get_variance() for r in ranges])
        
        # Gauss-Newton iterations
        for iteration in range(self.config.max_iterations):
            # Compute residuals and Jacobian
            r_gnss, J_gnss = self._compute_gnss_terms(x, gnss_states, id_to_idx)
            r_range, J_range = self._compute_range_terms(x, ranges, id_to_idx)
            
            # Weighted residuals
            r = np.concatenate([
                np.sqrt(W_gnss) * r_gnss,
                np.sqrt(W_range) * r_range
            ])
            
            # Weighted Jacobian
            J = np.vstack([
                np.sqrt(W_gnss)[:, None] * J_gnss,
                np.sqrt(W_range)[:, None] * J_range
            ])
            
            # Normal equations: J^T J Δx = -J^T r
            JTJ = J.T @ J
            JTr = J.T @ r
            
            # Solve (with regularization for numerical stability)
            try:
                delta_x = np.linalg.solve(JTJ + 1e-6 * np.eye(len(JTJ)), -JTr)
            except np.linalg.LinAlgError:
                # Singular matrix - use pseudoinverse
                delta_x = np.linalg.lstsq(JTJ, -JTr, rcond=None)[0]
            
            # Update
            x += delta_x
            
            # Check convergence
            step_size = np.linalg.norm(delta_x)
            if step_size < self.config.convergence_tol_m:
                break
        
        # Extract refined positions
        refined_positions = {}
        for aid, idx in id_to_idx.items():
            refined_positions[aid] = tuple(x[idx*3:(idx+1)*3])
        
        # Compute final residual
        r_gnss_final, _ = self._compute_gnss_terms(x, gnss_states, id_to_idx)
        r_range_final, _ = self._compute_range_terms(x, ranges, id_to_idx)
        
        residual_gnss = np.sqrt(np.sum(W_gnss * r_gnss_final**2))
        residual_range = np.sqrt(np.sum(W_range * r_range_final**2))
        final_residual = np.sqrt(residual_gnss**2 + residual_range**2)
        
        return refined_positions, iteration + 1, final_residual
    
    def _build_gnss_weights(
        self, 
        gnss_states: Dict[str, AnchorState],
        id_to_idx: Dict[str, int]
    ) -> np.ndarray:
        """Build weight vector from GNSS covariances."""
        n_anchors = len(id_to_idx)
        weights = np.zeros(n_anchors * 3)
        
        for aid, idx in id_to_idx.items():
            state = gnss_states[aid]
            # Use diagonal of covariance as weights (inverse variance)
            cov_diag = np.diag(state.pos_cov)
            weights[idx*3:(idx+1)*3] = self.config.gnss_weight_scale / (cov_diag + 1e-6)
        
        return weights
    
    def _compute_gnss_terms(
        self,
        x: np.ndarray,
        gnss_states: Dict[str, AnchorState],
        id_to_idx: Dict[str, int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute GNSS prior residuals and Jacobian."""
        n_anchors = len(id_to_idx)
        residuals = np.zeros(n_anchors * 3)
        jacobian = np.zeros((n_anchors * 3, n_anchors * 3))
        
        for aid, idx in id_to_idx.items():
            pos_pred = np.array(gnss_states[aid].pos_enu)
            pos_current = x[idx*3:(idx+1)*3]
            
            # Residual: difference from GNSS prior
            residuals[idx*3:(idx+1)*3] = pos_current - pos_pred
            
            # Jacobian: identity for each anchor
            jacobian[idx*3:(idx+1)*3, idx*3:(idx+1)*3] = np.eye(3)
        
        return residuals, jacobian
    
    def _compute_range_terms(
        self,
        x: np.ndarray,
        ranges: List[InterAnchorRangeReport],
        id_to_idx: Dict[str, int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute inter-anchor range residuals and Jacobian."""
        n_ranges = len(ranges)
        n_vars = len(x)
        residuals = np.zeros(n_ranges)
        jacobian = np.zeros((n_ranges, n_vars))
        
        for i, range_report in enumerate(ranges):
            idx_i = id_to_idx[range_report.anchor_id_i]
            idx_j = id_to_idx[range_report.anchor_id_j]
            
            pos_i = x[idx_i*3:(idx_i+1)*3]
            pos_j = x[idx_j*3:(idx_j+1)*3]
            
            # Computed distance
            diff = pos_i - pos_j
            dist_computed = np.linalg.norm(diff)
            
            # Residual: difference from measured range
            residuals[i] = dist_computed - range_report.distance_m
            
            # Jacobian: ∂||p_i - p_j||/∂p_i = (p_i - p_j) / ||p_i - p_j||
            if dist_computed > 1e-6:
                grad = diff / dist_computed
                jacobian[i, idx_i*3:(idx_i+1)*3] = grad
                jacobian[i, idx_j*3:(idx_j+1)*3] = -grad
        
        return residuals, jacobian
    
    def _estimate_fused_covariance(
        self, 
        gnss_state: AnchorState, 
        num_ranges: int
    ) -> np.ndarray:
        """
        Estimate covariance after fusion.
        
        Simple heuristic: reduce GNSS covariance based on number of ranges.
        More ranges = better constraint = lower uncertainty.
        
        Args:
            gnss_state: Original GNSS state
            num_ranges: Number of inter-anchor ranges used
            
        Returns:
            Estimated 3×3 covariance matrix
        """
        # Scale factor based on number of ranges
        # 0 ranges: scale = 1.0 (no improvement)
        # 3 ranges: scale = 0.5 (50% reduction in uncertainty)
        scale = 1.0 / (1.0 + 0.3 * num_ranges)
        
        return gnss_state.pos_cov * scale
    
    def _compute_fusion_quality(
        self,
        gnss_states: Dict[str, AnchorState],
        refined_states: Dict[str, AnchorState],
        ranges: List[InterAnchorRangeReport],
        residual: float
    ) -> float:
        """
        Compute fusion quality score (0-1).
        
        Considers:
        - Number of ranges used
        - GNSS quality of anchors
        - Residual magnitude
        - Convergence success
        
        Returns:
            Quality score between 0 (poor) and 1 (excellent)
        """
        # Component 1: Range count (more is better)
        range_score = min(1.0, len(ranges) / 3.0)  # 3 ranges = perfect
        
        # Component 2: GNSS quality (average)
        gnss_scores = []
        for state in gnss_states.values():
            if state.last_gnss_quality.is_excellent():
                gnss_scores.append(1.0)
            elif not state.last_gnss_quality.is_degraded():
                gnss_scores.append(0.7)
            else:
                gnss_scores.append(0.3)
        gnss_score = np.mean(gnss_scores) if gnss_scores else 0.5
        
        # Component 3: Residual (lower is better)
        residual_score = np.exp(-residual / 0.5)  # 0.5m characteristic scale
        
        # Combined score (weighted average)
        quality = 0.4 * range_score + 0.3 * gnss_score + 0.3 * residual_score
        
        return float(np.clip(quality, 0.0, 1.0))
    
    def _fallback_to_gnss(
        self,
        gnss_states: Dict[str, AnchorState],
        t_solve: float,
        num_ranges: int
    ) -> FusionResult:
        """Return GNSS-only result when fusion cannot proceed."""
        return FusionResult(
            t_solve=t_solve,
            refined_anchors=gnss_states.copy(),
            iterations=0,
            residual_norm=0.0,
            num_ranges_used=num_ranges,
            fusion_quality=0.5,  # Moderate quality without fusion
            fallback_to_gnss=True
        )
