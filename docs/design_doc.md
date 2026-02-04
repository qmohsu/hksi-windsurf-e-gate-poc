# Buoy Anchor Hub Subsystem Design Doc (v0.5)
## Phase 1: Exactly 3 Anchors, GNSS + UWB Only (No IMU)

## 1. Scope and Boundaries

### 1.1 In Scope (Phase 1)

**Buoy Anchor Hub (BAH)** on one buoy:

- Receives UWB range reports **(anchor ↔ tag)**
- Receives UWB inter-anchor range reports **(anchor ↔ anchor)** *(recommended if supported)*
- Receives GNSS position updates for anchors
- Maintains **time-varying** anchor positions in a session ENU frame
- Computes tag position estimates locally using **3-anchor multilateration**
- Computes start-line / gate metrics (moving-gate, based on buoy endpoints)
- Sends position estimates + quality + system health to the server

### 1.2 Explicitly Out of Scope (Phase 1)

- Any UWB+IMU fusion (both on anchors and on tags)
- Dead-reckoning using IMU during occlusion
- Coach UI, replay, analytics, RBAC, etc.

**Practical implication:** During UWB dropouts/occlusion, Phase 1 can only smooth/hold using a simple motion model, but cannot “bridge” gaps with IMU.

---

## 2. Phase 1 System Objective (Definition of "Done")

On-water, the BAH produces a stable stream of:

`PositionEstimate(tag_id, time, x, y, (z), quality)`

by combining:

- **Anchor positions** estimated from **GNSS + (optional) UWB inter-anchor ranging**
- **UWB ranges** between anchors and tag (exactly 3 anchors)

**AND** (Phase 1, start-line focus):

A stable stream of **gate metrics** derived from moving buoy endpoints:

- Signed distance-to-line
- Along-line progress
- Crossing event (time, direction, confidence)

---

## 3. Physical Topology (Phase 1)

### 3.1 Topology Constraint (Phase 1)

Because **anchor free mode is not available in Phase 1**, the system operates with:

- **Exactly 3 buoy anchors** deployed near the start area: `A0, A1, A2`
- 1 anchor hosts the BAH (Buoy Anchor Hub)
- The other 2 are FAs (Follower Anchors)
- Server reachable via 4G/Wi‑Fi (uplink handled by BAH)

**Important consequence:** With exactly 3 anchors, tag localization is **exactly-determined** (no measurement redundancy). This affects outlier handling (see §7.3).

### 3.2 Sensor Assumptions (Phase 1)

Each anchor has:

- **UWB module**
  - Provides anchor↔tag ranges
  - *(Recommended)* provides anchor↔anchor ranges
- **GNSS module**
  - Provides anchor position fixes and quality indicators
- IMU may physically exist, but software ignores it in Phase 1

---

## 4. Data Flow (Phase 1 Only)

### 4.1 Anchor ↔ Tag: UWB Range Reports

Each anchor produces range observations like:

> “Anchor Ai measured distance to Tag k = r meters at time t with quality q”

These are transmitted to the BAH over the local network.

### 4.2 Anchor ↔ Anchor: UWB Inter-Anchor Range Reports (Recommended)

If supported by the UWB mode/scheduling, anchors also exchange UWB ranges:

> “Anchor Ai measured distance to Anchor Aj = d_ij at time t with quality q”

These are used to **refine anchor geometry** between GNSS updates (see §6.5).

### 4.3 Anchor GNSS Updates → BAH

Each anchor provides GNSS fixes (low-rate). BAH uses these to maintain anchor positions.

**Two workable patterns for Phase 1:**

- **Pattern A (recommended):** Each anchor sends its GNSS fix to BAH
- **Pattern B:** BAH receives GNSS for itself + other anchors from an external source (less common)

BAH converts GNSS (lat/lon/alt) to a local ENU frame and uses those coordinates for anchor and tag estimation.

#### 4.3.1 Anchor Update Policy (MUST for On-Water)

- BAH accepts GNSS fixes **continuously** throughout a session (not a one-time init).
- Anchor positions are time-varying on water; **do not “freeze”** anchor coordinates after first receipt.
- If a GNSS fix is stale or low quality:
  - increase anchor uncertainty / reduce its weight in fusion
  - surface this in `HealthStatus`

### 4.4 BAH Localization → BAH Uplink

BAH produces `PositionEstimate` + `GateMetrics` and sends to server.

---

## 5. Coordinate Frames

### 5.1 Canonical Frame

Define a **local ENU frame**:

- **Origin** = BAH GNSS position at deployment start (or “session start”)

Convert all anchor GNSS fixes to ENU:

- `anchor_i → (E_i, N_i, U_i)`

Compute tag position in ENU:

- `tag_k → (E, N, U)` (or 2D if you choose to operate on water surface plane)

**Important clarification:**
The ENU origin is fixed at session start, but the BAH buoy will move. Therefore, the BAH buoy’s current position in ENU is generally not exactly (0,0,0). Treat BAH GNSS like any other anchor update (just with a known “session-origin”).

### 5.2 Time Conventions and Alignment (Phase 1)

- `t_gnss`: GNSS fix timestamp in UTC (preferred) or GNSS time-of-week (plus mapping to UTC)
- `t_bah`: BAH local monotonic timestamp used as the primary compute clock
- `t_range`: the timestamp used to solve a tag position. In Phase 1, define:
  - `t_range := t_bah_rx` (BAH receive time of the RangeReport), unless a reliable common timebase is available

**Goal:** When solving at `t_range`, use anchor positions predicted/interpolated to `t_range`
(not simply “latest GNSS fix”).

---

## 6. Anchor Location Estimation (Phase 1: GNSS + Optional Inter-Anchor UWB)

### 6.1 Anchor State (Per Anchor)

For each anchor `i ∈ {A0,A1,A2}` maintain:

- `pos_enu(t)` (E,N,(U))
- `vel_enu(t)` (optional but recommended for prediction)
- `gnss_quality` (fix type, HDOP, etc.)
- timestamp of last GNSS fix
- uncertainty / covariance estimate (used for weighting and health)

### 6.2 Baseline AnchorTracker (GNSS Only)

Because GNSS is noisy and low-rate, apply a lightweight tracker per anchor:

- constant-velocity Kalman filter (recommended) or EMA as minimum
- outputs:
  - filtered state at GNSS update time
  - predicted state at arbitrary `t_range`

### 6.3 AnchorTracker Interface (Normative)

For each anchor `i`, maintain `AnchorTracker_i` in ENU:

**`update_gnss(t_gnss_utc, pos_enu, gnss_quality)`**
- converts `gnss_quality` to measurement noise `R_gnss`
- updates filter state + covariance

**`predict(t_range)`** returns:
- `pos_enu_pred(t_range)`
- `cov_pred(t_range)` (uncertainty grows with prediction horizon)

### 6.4 Behavior Under Missing / Stale GNSS

If last GNSS fix is recent (within `T_hold`, e.g., 2–5 s):
- predict forward to `t_range`, but inflate uncertainty

If stale beyond `T_drop` (e.g., 10–20 s):
- keep predicting but mark anchor “degraded”
- optionally exclude inter-anchor fusion updates if they become inconsistent
- always surface `last_fix_age` and quality in `HealthStatus`

### 6.5 Anchor Network Fusion: GNSS + UWB Inter-Anchor Ranges (Recommended)

**Motivation:** With only 3 anchors, tag localization has no redundancy. Anchor geometry errors immediately become tag position errors. Inter-anchor UWB ranges can improve **relative anchor geometry** at higher rate than GNSS.

#### 6.5.1 What inter-anchor ranging can and cannot do

- Inter-anchor ranges alone define only a **relative** triangle shape (up to rotation/translation).
- GNSS is required to place that triangle in the **absolute ENU frame**.
- Therefore: **GNSS provides global reference**, inter-anchor UWB provides **relative constraints**.

#### 6.5.2 Recommended Phase-1 fusion architecture (Two-stage, implementable)

**Stage 1: per-anchor GNSS tracking**
- Run `AnchorTracker_i` (KF/EMA) to obtain:
  - `p_i^pred(t)` and `Σ_i^pred(t)` at any time `t`

**Stage 2: geometry refinement using inter-anchor ranges (small nonlinear WLS)**
At time `t` when you have inter-anchor ranges for pairs (0,1), (0,2), (1,2), solve:

Minimize over anchor positions `{p_0, p_1, p_2}`:

- **GNSS/Tracker prior term** (regularization):
  - keep `p_i` close to `p_i^pred(t)` with weight from `Σ_i^pred(t)`
- **UWB inter-anchor constraints**:
  - enforce `||p_i - p_j|| ≈ d_ij(t)` with weight from range variance

A simple cost function:

`J(p) = Σ_i ||p_i - p_i^pred||_{Σ_i^{-1}}^2  +  Σ_(i<j) (||p_i - p_j|| - d_ij)^2 / σ_ij^2`

Solve with 2–5 Gauss–Newton iterations (fast: only 6 unknowns in 2D, or 9 in 3D).

Then publish:
- `p_i^refined(t)` as the anchor position used by the tag solver
- an updated uncertainty (at minimum, track a scalar “anchor_quality”)

**Why this two-stage method is attractive in Phase 1**
- easy to implement and debug
- uses GNSS as both initial guess and continuous measurement
- leverages high-rate inter-anchor ranges to stabilize geometry between GNSS updates

#### 6.5.3 Alternative (single joint EKF / UKF)

You can maintain a single joint state:

`x = [p0,v0,p1,v1,p2,v2]`

and update with:
- GNSS position measurements `z_gnss_i = p_i + noise`
- inter-anchor range measurements `z_ij = ||p_i - p_j|| + noise`

This is elegant, but implementation is more sensitive (nonlinear updates, Jacobians, gating).
For Phase 1, the two-stage WLS refinement is typically the quickest route to a robust system.

#### 6.5.4 Gating and weighting rules (must-have)

Because UWB ranges can contain multipath outliers, apply these before using `d_ij`:

- sanity bounds: `d_min < d_ij < d_max`
- temporal consistency: `|d_ij(t) - d_ij(t-Δt)| < v_max * Δt + margin`
- quality flags from UWB (if available)

If an inter-anchor range fails gating, ignore it (fall back to GNSS-only anchor prediction for that step).

---

## 7. Tag Localization Engine (Phase 1: Exactly 3 Anchors, No IMU)

### 7.1 Inputs

At solve time `t_range` for tag `k`:

- Anchor positions at `t_range`:
  - `p_i(t_range)` from `AnchorTracker_i.predict(t_range)` **and/or** refined by §6.5
- Exactly 3 anchor↔tag ranges `{r_0, r_1, r_2}` plus their quality indicators
- Optional: an estimate of tag motion from a simple kinematic filter

### 7.2 Core Solver (Phase 1 Constraint)

With exactly **3 anchors**, the solver is exactly-determined.

Recommended approach:

- Solve in 2D (E,N) if you assume tag remains on water surface plane (most stable)
- Use a standard 3-anchor multilateration method (nonlinear solve)
- Apply weights if you have per-range variances, but note: **weights do not create redundancy**

### 7.3 Robustness and Outlier Handling (No Redundancy)

With only 3 anchors, you **cannot** do redundancy-based outlier removal (e.g., “drop one measurement and recompute”).
If one range is wrong, the position will be wrong.

Therefore, Phase 1 robustness should be implemented as:

1) **Range gating (hard rules)**
- drop negative / impossible ranges
- drop beyond operational distance
- drop low-quality flagged ranges (if quality is available)

2) **Solution plausibility checks (soft rules)**
- compare against predicted tag position from kinematic filter
- if the new solution implies impossible speed/acceleration, mark as low confidence or output no-fix
  - a safe policy is: **“reject the whole fix”** rather than trying to “repair” it

3) **Tag kinematic smoothing (allowed without IMU)**
- constant-velocity KF or α–β filter
- reduces jitter
- can hold briefly through missing fixes (short duration only)

### 7.4 Quality Score (Phase 1)

Quality should reflect:

- availability of all three ranges
- range quality flags
- anchor quality (GNSS and/or anchor fusion uncertainty)
- geometry health (e.g., triangle area / DOP-like heuristic)
- plausibility consistency vs tag motion model (innovation magnitude)

---

## 8. Gate Metrics (Moving Start Line)

Because buoys move, the start line is time-varying.

Define endpoints at time `t_range`:

- Left endpoint: `A_L(t_range)` (configured anchor id)
- Right endpoint: `A_R(t_range)` (configured anchor id)

Compute:

- `d_perp_signed`: signed perpendicular distance from tag to the line
- `s_along`: along-line coordinate (progress along the segment)
- crossing detection: sign change of `d_perp_signed` with interpolation
- `crossing_confidence`: based on tag uncertainty + endpoint uncertainty

---

## 9. Message Schemas (Phase 1 Minimal Set)

You can keep your length-prefixed framing, but payloads should be consistent and versioned.

### 9.1 Anchor → BAH: RangeReport (Anchor↔Tag)

- `schema_version`
- `anchor_id`
- `tag_id`
- `t_bah_rx` (required; used as `t_range` in Phase 1)
- `range_m`
- `quality` (optional but strongly recommended)
- `seq`

### 9.2 Anchor → BAH: InterAnchorRangeReport (Anchor↔Anchor, Recommended)

- `schema_version`
- `anchor_id_i`
- `anchor_id_j`
- `t_bah_rx`
- `range_m`
- `quality` (optional)
- `seq`

### 9.3 Anchor → BAH: AnchorGnssFix

- `schema_version`
- `anchor_id`
- `t_gnss_utc` (required if available)
- `lat`, `lon`, `alt`
- `fix_type`, `hdop`
- `t_bah_rx` (optional; diagnostics)

### 9.4 BAH → Server: PositionEstimate

- `schema_version`
- `tag_id`
- `t_bah` (solve time; equals `t_range`)
- `E`, `N` (and `U` if used)
- `quality_score`
- `anchors_used` (always 3 in Phase 1, but still record ids)
- (optional) uncertainty scalar or covariance

### 9.5 BAH → Server: GateMetrics

- `schema_version`
- `tag_id`
- `t_bah`
- `gate_left_anchor_id`, `gate_right_anchor_id`
- `d_perp_signed`
- `s_along`
- (optional) `crossing_detected`, `crossing_time_est`, `crossing_confidence`

### 9.6 BAH → Server: HealthStatus

- BAH CPU temp, uptime
- uplink status
- packet loss estimates
- per-anchor:
  - GNSS fix quality summary
  - last_fix_age
  - inter-anchor range health (optional: recent d_ij consistency vs predicted)

---

## 10. Failure Modes and Required Behavior (Phase 1)

### 10.1 Fewer than 3 valid anchor↔tag ranges

- Do not output a “fake” position
- Output “no-fix” with reason, or hold last valid position with degraded confidence (document your choice)

### 10.2 UWB range outlier on one of the 3 anchor↔tag links

- You cannot isolate which one is wrong (no redundancy)
- Preferred behavior:
  - reject the whole position fix if plausibility checks fail
  - report low confidence and/or hold last valid state briefly

### 10.3 GNSS poor quality or stale on one anchor

- Inflate that anchor’s uncertainty (lower weight in anchor fusion)
- If too stale, still predict but mark degraded and reduce overall position confidence
- Surface in `HealthStatus`

### 10.4 Inter-anchor range outliers (if inter-anchor ranging enabled)

- Gate outliers aggressively
- If inter-anchor ranges are unreliable, fall back to GNSS-only AnchorTracker for anchor positions

### 10.5 Uplink lost

- Continue computing locally
- Buffer last N seconds/minutes of `PositionEstimate` + `GateMetrics` for later upload (bounded)
