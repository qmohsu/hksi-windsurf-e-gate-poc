# Phase 1 Development Plan
## GNSS + UWB Only

This is the concrete build order I'd recommend.

---

## Milestone P1-A: "GNSS Anchor State Working (Continuous)"

### Deliverables

- Ingest `AnchorGnssFix` for each anchor continuously (no one-time init)
- Convert GNSS → ENU
- `AnchorTracker` per anchor (KF preferred; EMA minimum)
- Produce an internal "anchor state table" with:
  - `pos_enu_filtered`
  - `pos_enu_pred(t_now)`
  - `uncertainty / weight`
  - `last_fix_age` + `gnss_quality` flags

### Acceptance Criteria

- Anchor ENU positions update continuously
- `AnchorTracker` can predict/interpolate to arbitrary `t_range`
- A bad/stale GNSS fix lowers anchor weight (logged + surfaced)

---

## Milestone P1-B: "Multi-Anchor UWB Multilateration (Time-Aligned)"

### Deliverables

- Ingest `RangeReport` from multiple FAs
- Define `t_range := t_bah_rx` and solve per tag/time window
- For each solve, use `AnchorTracker_i.predict(t_range)` for anchor positions
- Solve using WLS multilateration with N≥3 (prefer N≥4)

### Acceptance Criteria

- Stable position for simulated tags in a lab geometry
- Uses >3 anchors when available (no forced `[:3]` truncation)
- Anchor motion between GNSS updates does not create large solution jumps

---

## Milestone P1-C: "Robustness Without IMU"

### Deliverables

- Outlier rejection loop (residual-based)
- Quality score output
- Simple kinematic filter (no IMU)

### Acceptance Criteria

- Injected range outliers do not cause large jumps
- Output confidence drops gracefully instead of producing nonsense

---

## Milestone P1-D: "Gate Metrics (Moving Start Line)"

### Deliverables

- Compute gate endpoints from configured anchor IDs using predicted positions at `t_range`
- Output `GateMetrics` (`d_perp_signed`, crossing detection, confidence)
- Buffer `GateMetrics` during uplink outages (like `PositionEstimate`)

### Acceptance Criteria

- Signed distance-to-line is stable on water (no obvious GNSS jitter artifacts)
- Crossing events are detected consistently with reasonable confidence behavior

---

## Milestone P1-E: "BAH Uplink to Server"

### Deliverables

- Send `PositionEstimate` + `GateMetrics` + `HealthStatus` to server endpoint
- Retry and buffering for uplink dropouts

### Acceptance Criteria

- Server receives a clean real-time stream
- System continues operating through temporary uplink failures