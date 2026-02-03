Buoy Anchor Hub Subsystem Design Doc (v0.3) — Phase 1: GNSS + UWB only
========================================================================

1) Scope and boundaries
-----------------------

In scope (Phase 1)

- Buoy Anchor Hub (BAH) on one buoy:
  - Receives UWB range reports from multiple anchors (Follower Anchors, FA)
  - Receives GNSS position updates for anchors (could be from each anchor's GNSS,
    or centralized if you decide)
  - Computes tag position estimates locally using UWB multilateration
  - Sends position estimates + quality + system health to the server

Explicitly out of scope (Phase 1)

- Any UWB+IMU fusion (both on anchors and on tags)
- Any dead-reckoning during occlusion using IMU
- Coach UI, replay, analytics, RBAC, etc.

Practical implication: during UWB dropouts/occlusion, Phase 1 can only
smooth/hold using a simple motion model, but cannot "bridge" gaps with IMU.

2) Phase 1 system objective (definition of "done")
--------------------------------------------------

On-water, the BAH can produce a stable stream of:

PositionEstimate(tag_id, time, x,y,(z), quality)

by combining:

- Anchor positions from GNSS (low-rate, global reference)
- UWB ranges (high-rate, local relative measurement)

This directly mirrors the concept of low-rate GNSS + high-rate UWB and keeping
early versions simple.

Reference: HKSI_SRFS_presentation_20260123...

3) Physical topology (Phase 1)
------------------------------

Minimum topology

- 3–8 buoy anchors deployed near the start area
- 1 is the BAH
- Others are FAs
- Server reachable via 4G/Wi‑Fi (uplink handled by BAH)

Sensor assumptions (Phase 1)

Each anchor has:

- UWB module (produces ranges)
- GNSS module (produces anchor position)
- IMU may physically exist (per later phases), but software ignores it in
  Phase 1.

4) Data flow (Phase 1 only)
---------------------------

4.1 FA → BAH: UWB range reports

Each FA produces range observations like:

"FA_i measured distance to Tag_k = r meters at time t with quality q"

These are transmitted to the BAH over the local network.

4.2 Anchor GNSS updates → BAH

Each anchor provides GNSS fixes (low-rate). BAH uses these to maintain anchor
positions.

Two workable patterns for Phase 1:

- Pattern A (recommended): each anchor sends its GNSS fix to BAH
- Pattern B: BAH receives GNSS for itself + other anchors from an external
  source (less common)

BAH converts GNSS (lat/lon/alt) to a local ENU frame and uses those coordinates
for multilateration.

4.3 BAH localization → BAH uplink

BAH produces PositionEstimate and sends to server.

5) Coordinate frames (Phase 1)
------------------------------

Canonical frame

Define local ENU frame:

- origin = BAH GNSS position at deployment start (or "session start")
- Convert all anchor GNSS fixes to ENU:
  - anchor_i → (E_i, N_i, U_i)
- Compute tag position in ENU:
  - tag_k → (E, N, U) (or 2D if you choose to operate on water surface plane)

This is intentionally simple and matches your current code's coordinate
conversion direction.

6) Anchor state model (Phase 1: GNSS only)
------------------------------------------

Each anchor maintains:

- pos_enu(t) from GNSS
- gnss_quality: fix type, HDOP, etc.
- timestamp

Smoothing

Because GNSS is noisy and low-rate, apply a lightweight smoother:

- exponential moving average (EMA) or small Kalman filter on anchor position

Buoy drift handling (Phase 1)

Drift can be "handled" in a limited sense by continuous GNSS updates +
smoothing.

Full autocalibration (and drift modeling using IMU) is deferred to later phases,
but you still get a workable baseline with GNSS alone. This is consistent with
the staged concept of gradually increasing robustness.

Reference: Final SRFS Application Form_260...

7) Localization engine (Phase 1: UWB multilateration, no IMU)
-------------------------------------------------------------

7.1 Inputs

For each tag k over a short time window:

- Anchor positions: {(E_i, N_i, U_i)}
- Range observations: {r_i} with quality weights

7.2 Core solver (recommended)

Use N-anchor weighted least squares (WLS) multilateration:

- supports 3–8 anchors
- uses weights based on measurement quality (RSSI / reported quality flags)

7.3 Robustness (still needed even without IMU)

Implement outlier handling because marine multipath/NLOS still exists:

Range gating:

- drop negative / impossible values
- drop beyond max operational distance

Residual-based rejection:

- solve once → compute residuals → drop worst residual(s) → re-solve

Output a quality_score based on:

- number of anchors used
- residual statistics
- GNSS quality of anchors

7.4 Smoothing / tracking (allowed without IMU)

Even without IMU, you should still run a simple tracker:

- constant-velocity Kalman filter or α–β filter
- helps reduce jitter
- helps survive brief missing measurements by prediction (short duration only)

Important: this is not IMU fusion; it's just a kinematic smoothing layer.

8) Message schemas (Phase 1 minimal set)
----------------------------------------

You can keep your length-prefixed framing, but it's worth making payloads
consistent and versioned (protobuf preferred).

8.1 FA → BAH: RangeReport

- schema_version
- anchor_id
- tag_id
- t_anchor
- range_m
- quality (optional but strongly recommended)
- seq

8.2 Anchor → BAH: AnchorGnssFix

- schema_version
- anchor_id
- t_gnss
- lat, lon, alt
- fix_type, hdop (or equivalent)

8.3 BAH → Server: PositionEstimate

- schema_version
- tag_id
- t_bah
- E, N, U (or E, N)
- quality_score
- anchors_used[]
- residual_stats

8.4 BAH → Server: HealthStatus

- BAH CPU temp, uptime
- uplink status
- FA packet loss estimates
- GNSS fix quality summary (anchors)

9) Failure modes and required behavior (Phase 1)
------------------------------------------------

9.1 If fewer than 3 anchors available for a tag

- Do not output a "fake" position
- Output "no-fix" state with reason, or hold last valid position with degraded
  confidence (your choice, but document it)

9.2 GNSS poor quality on one anchor

- Reduce its weight (or temporarily exclude it)
- Report anchor GNSS quality in HealthStatus

9.3 UWB outlier burst

- Robust rejection should drop bad ranges
- If instability persists: output low confidence and/or pause estimates rather
  than jumping

9.4 Uplink lost

- Continue computing locally
- Buffer last N seconds/minutes of PositionEstimate for later upload (bounded)

Revised Phase 1 Development Plan (GNSS + UWB only)
--------------------------------------------------

This is the concrete build order I'd recommend.

Milestone P1-A — "GNSS anchor state working"

Deliverables

- Ingest AnchorGnssFix for each anchor
- Convert GNSS → ENU
- Apply smoothing
- Produce an internal "anchor state table" with quality flags

Acceptance

- Anchor ENU positions update continuously
- A bad GNSS fix visibly lowers anchor confidence (logged + surfaced)

Milestone P1-B — "Multi-anchor UWB multilateration"

Deliverables

- Ingest RangeReport from multiple FAs
- Group by tag_id + time window
- Solve using WLS multilateration with N≥3

Acceptance

- Stable position for simulated tags in a lab geometry
- Uses >3 anchors when available (no forced [:3] truncation)

Milestone P1-C — "Robustness without IMU"

Deliverables

- Outlier rejection loop (residual-based)
- Quality score output
- Simple kinematic filter (no IMU)

Acceptance

- Injected range outliers do not cause large jumps
- Output confidence drops gracefully instead of producing nonsense

Milestone P1-D — "BAH uplink to server"

Deliverables

- Send PositionEstimate + HealthStatus to server endpoint
- Retry and buffering for uplink dropouts

Acceptance

- Server receives a clean real-time stream
- System continues operating through temporary uplink failures
