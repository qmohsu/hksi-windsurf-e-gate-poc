"""
Anchor Tracker with Kalman Filter.

Maintains per-anchor state using constant-velocity Kalman filter to track
GNSS position updates and predict anchor positions at arbitrary solve times.

Implements Design Doc Section 6 (Anchor Location Estimation).
Milestone P1-A: Continuous GNSS anchor state.

Reference:
- Section 6.2: Baseline AnchorTracker (GNSS Only)
- Section 6.3: AnchorTracker Interface (normative)
- Section 6.4: Behavior Under Missing / Stale GNSS
"""

import time
import logging
from typing import Tuple, Optional
import numpy as np

from .gnss_quality import GNSSQuality, quality_to_measurement_covariance
from .anchor_state import AnchorState
from bah_core.metrics import get_metrics

logger = logging.getLogger(__name__)


class AnchorTracker:
    """
    Per-anchor Kalman filter for continuous GNSS state tracking.
    
    State vector (6D constant-velocity model):
        x = [E, N, U, vE, vN, vU]^T
        
    Measurement (3D position from GNSS):
        z = [E, N, U]^T
        
    Usage:
        tracker = AnchorTracker("A0")
        tracker.update_gnss(t_gnss, (e, n, u), quality)
        state = tracker.predict(t_range)
    """
    
    # Configuration constants (per Design Doc Section 6.4)
    T_HOLD = 5.0      # Hold prediction up to 5 seconds
    T_DROP = 20.0     # Mark degraded after 20 seconds
    
    # Process noise parameters
    Q_POS = 0.1 ** 2  # Position process noise (m^2)
    Q_VEL = 0.5 ** 2  # Velocity process noise ((m/s)^2)
    
    def __init__(self, anchor_id: str):
        """
        Initialize anchor tracker.
        
        Args:
            anchor_id: Anchor identifier (e.g., "A0", "A1")
        """
        self.anchor_id = anchor_id
        
        # State vector [E, N, U, vE, vN, vU]
        self.x = np.zeros(6)
        
        # State covariance (6x6)
        self.P = np.eye(6) * 100.0  # Large initial uncertainty
        
        # Last GNSS update
        self.last_gnss_time: Optional[float] = None
        self.last_gnss_quality: Optional[GNSSQuality] = None
        
        # Initialization flag
        self.initialized = False
        
        # Metrics
        self.metrics = get_metrics()
        
        logger.info(f"AnchorTracker initialized for {anchor_id}")
    
    def update_gnss(
        self,
        t_gnss: float,
        pos_enu: Tuple[float, float, float],
        quality: GNSSQuality
    ):
        """
        Update filter with new GNSS measurement.
        
        Args:
            t_gnss: GNSS measurement time (seconds, BAH monotonic clock)
            pos_enu: Position in ENU frame (E, N, U) meters
            quality: GNSS quality metrics
        """
        self.metrics.increment('gnss_updates')
        
        if not quality.is_valid():
            logger.warning(f"{self.anchor_id}: Invalid GNSS quality, skipping update")
            self.metrics.increment_drop('gnss_poor_quality')
            return
        
        # First update: initialize state
        if not self.initialized:
            self._initialize_from_gnss(pos_enu, quality)
            self.last_gnss_time = t_gnss
            self.last_gnss_quality = quality
            logger.info(f"{self.anchor_id}: Initialized at {pos_enu}")
            return
        
        # Predict to measurement time
        dt = t_gnss - self.last_gnss_time
        if dt < 0:
            logger.warning(f"{self.anchor_id}: Out-of-order GNSS (dt={dt:.3f}s), skipping")
            self.metrics.increment_drop('out_of_order')
            return
        
        if dt > 0:
            self._predict_step(dt)
        
        # Kalman update
        self._update_step(pos_enu, quality)
        
        # Save update info
        self.last_gnss_time = t_gnss
        self.last_gnss_quality = quality
        
        logger.debug(f"{self.anchor_id}: GNSS update at t={t_gnss:.3f}, "
                    f"pos=({pos_enu[0]:.2f}, {pos_enu[1]:.2f}, {pos_enu[2]:.2f})")
    
    def predict(self, t_range: float) -> AnchorState:
        """
        Predict anchor state at arbitrary time.
        
        Args:
            t_range: Solve time (seconds, BAH monotonic clock)
            
        Returns:
            AnchorState with predicted position, velocity, uncertainty
        """
        if not self.initialized:
            logger.warning(f"{self.anchor_id}: Not initialized, returning invalid state")
            return AnchorState(
                anchor_id=self.anchor_id,
                time=t_range,
                pos_enu=(0.0, 0.0, 0.0),
                is_valid=False,
            )
        
        # Prediction horizon
        dt = t_range - self.last_gnss_time
        
        # Predict forward
        if dt > 0:
            x_pred, P_pred = self._predict_state(dt)
        else:
            # Use current state (t_range <= last_gnss_time)
            x_pred = self.x.copy()
            P_pred = self.P.copy()
        
        # Extract position and velocity
        pos_enu = tuple(x_pred[0:3])
        vel_enu = tuple(x_pred[3:6])
        
        # Position covariance (3x3 upper-left block)
        pos_cov = P_pred[0:3, 0:3]
        
        # Determine validity and degradation
        is_valid = True
        is_degraded = False
        
        if self.last_gnss_time is None:
            is_valid = False
        elif dt > self.T_DROP:
            is_degraded = True
            self.metrics.increment('stale_fixes')
            logger.debug(f"{self.anchor_id}: Stale fix (age={dt:.1f}s > {self.T_DROP}s)")
        elif dt > self.T_HOLD:
            is_degraded = True
        
        self.metrics.record_histogram('prediction_horizon_s', dt)
        
        return AnchorState(
            anchor_id=self.anchor_id,
            time=t_range,
            pos_enu=pos_enu,
            vel_enu=vel_enu,
            pos_cov=pos_cov,
            last_gnss_time=self.last_gnss_time,
            last_gnss_quality=self.last_gnss_quality,
            is_valid=is_valid,
            is_degraded=is_degraded,
        )
    
    def _initialize_from_gnss(self, pos_enu: Tuple[float, float, float], quality: GNSSQuality):
        """Initialize state from first GNSS measurement."""
        # Set position, zero velocity
        self.x[0:3] = pos_enu
        self.x[3:6] = 0.0
        
        # Set initial covariance
        # Position uncertainty from GNSS quality
        R = quality_to_measurement_covariance(quality)
        self.P[0:3, 0:3] = R
        
        # Velocity uncertainty (assume 0.5 m/s initial uncertainty)
        self.P[3:6, 3:6] = np.eye(3) * (0.5 ** 2)
        
        self.initialized = True
    
    def _predict_step(self, dt: float):
        """
        Predict state forward by dt (in-place update).
        
        Args:
            dt: Time step in seconds
        """
        x_pred, P_pred = self._predict_state(dt)
        self.x = x_pred
        self.P = P_pred
    
    def _predict_state(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict state forward by dt (pure function).
        
        Args:
            dt: Time step in seconds
            
        Returns:
            (x_pred, P_pred) predicted state and covariance
        """
        # State transition matrix (constant velocity)
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt  # Position += velocity * dt
        
        # Predict state
        x_pred = F @ self.x
        
        # Process noise covariance
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * self.Q_POS  # Position noise
        Q[3:6, 3:6] = np.eye(3) * self.Q_VEL  # Velocity noise
        
        # Predict covariance
        P_pred = F @ self.P @ F.T + Q
        
        return x_pred, P_pred
    
    def _update_step(self, z: Tuple[float, float, float], quality: GNSSQuality):
        """
        Kalman update step with GNSS measurement.
        
        Args:
            z: Measurement (E, N, U) in meters
            quality: GNSS quality for measurement noise
        """
        # Measurement matrix (observe position only)
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)
        
        # Measurement noise covariance
        R = quality_to_measurement_covariance(quality)
        
        # Innovation
        z_vec = np.array(z)
        y = z_vec - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        # Record innovation magnitude for diagnostics
        innovation_mag = np.linalg.norm(y)
        self.metrics.record_histogram('gnss_innovation_m', innovation_mag)
        
        logger.debug(f"{self.anchor_id}: Innovation magnitude: {innovation_mag:.3f}m")
    
    def get_current_state(self) -> AnchorState:
        """
        Get current state (no prediction).
        
        Returns:
            AnchorState at last GNSS update time
        """
        if not self.initialized:
            return AnchorState(
                anchor_id=self.anchor_id,
                time=0.0,
                pos_enu=(0.0, 0.0, 0.0),
                is_valid=False,
            )
        
        return AnchorState(
            anchor_id=self.anchor_id,
            time=self.last_gnss_time,
            pos_enu=tuple(self.x[0:3]),
            vel_enu=tuple(self.x[3:6]),
            pos_cov=self.P[0:3, 0:3].copy(),
            last_gnss_time=self.last_gnss_time,
            last_gnss_quality=self.last_gnss_quality,
            is_valid=True,
            is_degraded=False,
        )
