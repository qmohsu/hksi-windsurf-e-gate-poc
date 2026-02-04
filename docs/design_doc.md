# Buoy Anchor Hub Subsystem Design Doc (v0.4)
## Phase 1: GNSS + UWB only

## 1. Scope and Boundaries

### 1.1 In Scope (Phase 1)

**Buoy Anchor Hub (BAH)** on one buoy:

- Receives UWB range reports from multiple anchors (Follower Anchors, FA)
- Receives GNSS position updates for anchors (could be from each anchor's GNSS, or centralized if you decide)
- Computes tag position estimates locally using UWB multilateration
- Computes start-line / gate metrics (moving-gate, based on buoy endpoints)
- Sends position estimates + quality + system health to the server

### 1.2 Explicitly Out of Scope (Phase 1)

- Any UWB+IMU fusion (both on anchors and on tags)
- Any dead-reckoning during occlusion using IMU
- Coach UI, replay, analytics, RBAC, etc.

**Practical implication:** During UWB dropouts/occlusion, Phase 1 can only smooth/hold using a simple motion model, but cannot "bridge" gaps with IMU.

## 2. Phase 1 System Objective (Definition of "Done")

On-water, the BAH can produce a stable stream of:

`PositionEstimate(tag_id, time, x, y, (z), quality)`

by combining:

- **Anchor positions from GNSS** (low-rate, global reference)
- **UWB ranges** (high-rate, local relative measurement)

**AND** (Phase 1, start-line focus):

A stable stream of **gate metrics** derived from moving buoy endpoints:

- Signed distance-to-line
- Along-line progress
- Crossing event (time, direction, confidence)

This directly mirrors the concept of low-rate GNSS + high-rate UWB and keeping early versions simple.

**Reference:** HKSI_SRFS_presentation_20260123...

## 3. Physical Topology (Phase 1)

### 3.1 Minimum Topology

- 3–8 buoy anchors deployed near the start area
- 1 is the BAH (Buoy Anchor Hub)
- Others are FAs (Follower Anchors)
- Server reachable via 4G/Wi-Fi (uplink handled by BAH)

### 3.2 Sensor Assumptions (Phase 1)

Each anchor has:

- **UWB module** (produces ranges)
- **GNSS module** (produces anchor position)
- **IMU** may physically exist (per later phases), but software ignores it in Phase 1

## 4. Data Flow (Phase 1 Only)

### 4.1 FA → BAH: UWB Range Reports

Each FA produces range observations like:

> "FA_i measured distance to Tag_k = r meters at time t with quality q"

These are transmitted to the BAH over the local network.

### 4.2 Anchor GNSS Updates → BAH

Each anchor provides GNSS fixes (low-rate). BAH uses these to maintain anchor positions.

**Two workable patterns for Phase 1:**

- **Pattern A (recommended):** Each anchor sends its GNSS fix to BAH
- **Pattern B:** BAH receives GNSS for itself + other anchors from an external source (less common)

BAH converts GNSS (lat/lon/alt) to a local ENU frame and uses those coordinates for multilateration.

#### 4.2.1 Anchor Update Policy (MUST for On-Water)

BAH accepts GNSS fixes **continuously** throughout a session (not a one-time init).

Anchor positions are time-varying on water; **do not "freeze"** anchor coordinates after first receipt.

If a GNSS fix is stale or low quality:

- Down-weight that anchor in multilateration, or exclude it temporarily
- Surface this in `HealthStatus`

### 4.3 BAH Localization → BAH Uplink

BAH produces `PositionEstimate` and sends to server.

## 5. Coordinate Frames (Phase 1)

### 5.1 Canonical Frame

Define **local ENU frame:**

- **Origin** = BAH GNSS position at deployment start (or "session start")

Convert all anchor GNSS fixes to ENU:

- `anchor_i → (E_i, N_i, U_i)`

Compute tag position in ENU:

- `tag_k → (E, N, U)` (or 2D if you choose to operate on water surface plane)

**Important clarification:**

The ENU origin is fixed at session start, but the BAH buoy will move. Therefore, the BAH buoy's current position in ENU is generally not exactly (0,0,0). Treat BAH GNSS like any other anchor update (just with a known "session-origin").

### 5.2 Time Conventions and Alignment (Phase 1)

- **`t_gnss`:** GNSS fix timestamp in UTC (preferred) or GNSS time-of-week, plus a mapping to UTC
- **`t_anchor`:** Anchor device local timestamp (optional; useful for debugging only)
- **`t_bah`:** BAH local monotonic timestamp used as the primary compute clock
- **`t_range`:** The timestamp used to solve a tag position. In Phase 1, define:
  - `t_range := t_bah_rx` (BAH receive time of the RangeReport), unless a reliable common timebase is available

**Goal:** When solving at `t_range`, use anchor positions predicted/interpolated to `t_range` (not simply "latest GNSS fix").

**Note:** Perfect clock sync is not required in Phase 1, but a consistent definition of `t_range` is required.

## 6. Anchor State Model (Phase 1: GNSS Only)

Each anchor maintains:

- `pos_enu(t)` derived from GNSS
- `gnss_quality`: fix type, HDOP, etc.
- Timestamp of last fix
- (Recommended) Covariance / uncertainty estimate for weighting

### 6.1 Smoothing / Tracking (Recommended Baseline)

Because GNSS is noisy and low-rate, apply a lightweight tracker per anchor:

- **Constant-velocity Kalman filter** (recommended) or EMA as minimum

Output both:

- Filtered state at GNSS update time
- Predicted state at arbitrary `t_range`

### 6.2 AnchorTracker Interface (Normative)

For each anchor `i`, maintain `AnchorTracker_i` in ENU:

**`update_gnss(t_gnss_utc, pos_enu, gnss_quality)`:**

- Converts `gnss_quality` to measurement noise (R)
- Updates filter state + covariance

**`predict(t_range)`** returns:

- `pos_enu_pred(t_range)`
- `cov_pred(t_range)` (uncertainty grows with prediction horizon)

### 6.3 Behavior Under Missing / Stale GNSS

**If the last GNSS fix is recent** (within `T_hold`, e.g., 2–5 s):

- Predict forward to `t_range`, but inflate uncertainty

**If stale beyond `T_drop`** (e.g., 10–20 s):

- Exclude anchor from multilateration

**Always log:** `anchor_id`, `last_fix_age`, `gnss_quality`, include/exclude decision

### 6.4 Buoy Drift Handling (Phase 1)

Drift is handled by continuous GNSS updates + per-anchor tracking. Full autocalibration (and drift modeling using IMU) is deferred to later phases, but GNSS+tracking is a workable baseline on water.

**Reference:** Final SRFS Application Form_260...

## 7. Localization Engine (Phase 1: UWB Multilateration, No IMU)

### 7.1 Inputs

For each tag `k` over a short time window at solve time `t_range`:

- **Anchor positions at `t_range`:** `{(E_i(t_range), N_i(t_range), U_i(t_range))}`
  - Obtained from `AnchorTracker_i.predict(t_range)`
- **Range observations:** `{r_i}` with quality weights
- **Optional:** Anchor uncertainty from `cov_pred(t_range)` to down-weight poor anchors

### 7.2 Core Solver (Recommended)

Use **N-anchor weighted least squares (WLS) multilateration:**

- Supports 3–8 anchors
- Uses weights based on:
  - UWB measurement quality (RSSI / reported quality flags)
  - GNSS/anchor uncertainty (anchors with poor GNSS should contribute less)

**Operational rule:**

- Solve with N ≥ 4 by default when available (robustness)
- Allow N=3 only as a fallback, with reduced `quality_score`

### 7.3 Robustness (Still Needed Even Without IMU)

Implement outlier handling because marine multipath/NLOS still exists:

**Range gating:**

- Drop negative / impossible values
- Drop beyond max operational distance

**Residual-based rejection:**

- Solve once → compute residuals → drop worst residual(s) → re-solve (requires redundancy)

**Output a `quality_score` based on:**

- Number of anchors used
- Residual statistics
- GNSS quality / uncertainty of anchors
- Geometry health (e.g., anchor spread / dilution heuristic)

### 7.4 Smoothing / Tracking (Allowed Without IMU)

Even without IMU, run a simple tracker per tag:

- **Constant-velocity Kalman filter** or **α–β filter**
- Reduces jitter
- Survives brief missing measurements by prediction (short duration only)

**Important:** This is not IMU fusion; it's just a kinematic smoothing layer.

### 7.5 Gate Metrics (Moving Start Line as First-Class Output)

Because buoys move, the start line is time-varying.

**Define endpoints at time `t_range`:**

- **Left endpoint:** `A_L(t_range)` (anchor id configured)
- **Right endpoint:** `A_R(t_range)` (anchor id configured)

**Compute gate metrics for each `PositionEstimate`:**

- **`d_perp_signed`:** Signed perpendicular distance from tag to the line
  - Sign convention: choose and document (e.g., negative = pre-start, positive = post-start)
- **`s_along`:** Along-line coordinate (progress along the line segment)
- **`crossing_detected`:** Sign change of `d_perp_signed` across time with interpolation
- **`crossing_time_est`:** Interpolated crossing time (within the BAH timebase)
- **`crossing_confidence`:** Based on position uncertainty + endpoint uncertainty

These metrics are used for coaching and event detection and should be generated even if the server is temporarily unreachable (buffered like `PositionEstimate`).

## 8. Message Schemas (Phase 1 Minimal Set)

You can keep your length-prefixed framing, but it's worth making payloads consistent and versioned (protobuf preferred).

### 8.1 FA → BAH: RangeReport

- `schema_version`
- `anchor_id`
- `tag_id`
- `t_bah_rx` (required; BAH receive time, used for `t_range` in Phase 1)
- `t_anchor_tx` (optional; anchor local time, debugging only)
- `range_m`
- `quality` (optional but strongly recommended)
- `seq`

### 8.2 Anchor → BAH: AnchorGnssFix

- `schema_version`
- `anchor_id`
- `t_gnss_utc` (required if available; otherwise provide best-effort UTC mapping)
- `lat`, `lon`, `alt`
- `fix_type`, `hdop` (or equivalent)
- `t_bah_rx` (optional; time BAH received this GNSS message, for diagnostics)

### 8.3 BAH → Server: PositionEstimate

- `schema_version`
- `tag_id`
- `t_bah` (solve time; equals `t_range`)
- `E`, `N`, `U` (or `E`, `N`)
- `quality_score`
- `anchors_used[]`
- `residual_stats`
- (Optional) `position_cov` or uncertainty scalar

### 8.4 BAH → Server: GateMetrics (Recommended)

- `schema_version`
- `tag_id`
- `t_bah`
- `gate_left_anchor_id`, `gate_right_anchor_id`
- `d_perp_signed`
- `s_along`
- (Optional) `crossing_detected`, `crossing_time_est`, `crossing_confidence`

### 8.5 BAH → Server: HealthStatus

- BAH CPU temp, uptime
- Uplink status
- FA packet loss estimates
- GNSS fix quality summary (anchors)
- Anchor staleness summary (`last_fix_age` per anchor)

## 9. Failure Modes and Required Behavior (Phase 1)

### 9.1 If Fewer Than 3 Anchors Available for a Tag

- Do not output a "fake" position
- Output "no-fix" state with reason, or hold last valid position with degraded confidence (your choice, but document it)

### 9.2 GNSS Poor Quality or Stale on One Anchor

- Reduce its weight (or temporarily exclude it)
- Report anchor GNSS quality + staleness in `HealthStatus`
- Ensure `AnchorTracker` uncertainty grows with time since last fix

### 9.3 UWB Outlier Burst

- Robust rejection should drop bad ranges (needs redundancy)
- If instability persists: output low confidence and/or pause estimates rather than jumping

### 9.4 Uplink Lost

- Continue computing locally
- Buffer last N seconds/minutes of `PositionEstimate` + `GateMetrics` for later upload (bounded)