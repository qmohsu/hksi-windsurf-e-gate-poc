# Phase 1 Development Plan (v0.5)
## Exactly 3 Anchors, GNSS + UWB Only (No IMU)

This plan assumes Phase 1 runs with **exactly three anchors** (`A0,A1,A2`) and therefore **no redundancy** for tag multilateration.

---

## Milestone P1-A: "Continuous GNSS Anchor State (Per-Anchor)"

### Deliverables

- Ingest `AnchorGnssFix` for each anchor continuously (no one-time init)
- Convert GNSS → ENU (session ENU origin fixed at start)
- `AnchorTracker_i` per anchor (KF preferred; EMA minimum)
- Internal anchor state table:
  - `pos_enu_filtered`
  - `pos_enu_pred(t_now)`
  - uncertainty / weight
  - `last_fix_age` + `gnss_quality` flags

### Acceptance Criteria

- Anchor ENU positions update continuously
- `AnchorTracker` can predict/interpolate to arbitrary `t_range`
- Bad/stale GNSS fix inflates uncertainty and is surfaced in `HealthStatus`

---

## Milestone P1-B: "Inter-Anchor UWB Ranging + Anchor Geometry Fusion (Recommended)"

*(Skip this milestone only if the UWB mode cannot provide inter-anchor ranges in Phase 1.)*

### Deliverables

- Ingest `InterAnchorRangeReport` for pairs (A0–A1, A0–A2, A1–A2)
- Range gating rules for inter-anchor data (sanity bounds + temporal consistency)
- Implement **Anchor Network Fusion** using:
  - GNSS/Tracker predictions as priors
  - inter-anchor ranges as constraints
- Output refined anchor positions at time `t`:
  - `p_i_refined(t)` to feed the tag solver
  - a scalar `anchor_quality` or (better) covariance proxy

### Acceptance Criteria

- With stationary anchors (lab test), refined anchor triangle is stable and matches tape-measured distances
- With simulated GNSS noise, inter-anchor fusion reduces relative geometry jitter vs GNSS-only
- If inter-anchor ranges are missing or fail gating, system cleanly falls back to GNSS-only prediction

---

## Milestone P1-C: "3-Anchor Tag Multilateration (Time-Aligned)"

### Deliverables

- Ingest anchor↔tag `RangeReport`
- Define `t_range := t_bah_rx`
- At each solve, obtain anchor positions at `t_range` using:
  - `AnchorTracker_i.predict(t_range)` and/or `p_i_refined(t_range)` from P1-B
- Solve tag position using **exactly 3 anchors**
  - prefer 2D solve on water surface plane (stable)
- Output `PositionEstimate` with a first-pass `quality_score`

### Acceptance Criteria

- Stable position for simulated tags in lab geometry (no “one-time anchor init” issues)
- If any of the 3 ranges is invalid/missing → output **no-fix** (or a documented hold policy)
- Tag position updates do not show large jumps when anchors move smoothly between GNSS updates

---

## Milestone P1-D: "Phase-1 Robustness Without Redundancy"

Because tag localization has **no redundancy** with 3 anchors, Phase 1 robustness is implemented via gating + plausibility checks (not drop-one-and-recompute style outlier removal).

### Deliverables

- Range gating (hard rules) for anchor↔tag ranges
- Tag kinematic filter (KF or α–β) to reduce jitter
- Plausibility checks vs motion model:
  - if implied speed/acceleration is impossible → reject fix or output degraded confidence
- Quality score update to include:
  - anchor quality (GNSS + fusion uncertainty)
  - triangle geometry health
  - innovation magnitude vs filter prediction

### Acceptance Criteria

- Injected single-range outliers cause **rejected/no-fix** rather than “teleporting” tag positions
- During brief range dropouts, filter can hold smoothly for a short, bounded time (document the limits)

---

## Milestone P1-E: "Gate Metrics (Moving Start Line)"

### Deliverables

- Compute gate endpoints from configured anchor IDs using predicted/refined positions at `t_range`
- Output `GateMetrics` (`d_perp_signed`, crossing detection, confidence)
- Buffer `GateMetrics` during uplink outages (like `PositionEstimate`)

### Acceptance Criteria

- Signed distance-to-line is stable on water (no obvious GNSS jitter artifacts)
- Crossing events are detected consistently with reasonable confidence behavior

---

## Milestone P1-F: "BAH Uplink to Server"

### Deliverables

- Send `PositionEstimate` + `GateMetrics` + `HealthStatus` to server endpoint
- Retry and buffering for uplink dropouts

### Acceptance Criteria

- Server receives a clean real-time stream
- System continues operating through temporary uplink failures
