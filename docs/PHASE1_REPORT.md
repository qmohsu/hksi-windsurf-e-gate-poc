# Phase 1: UWB Positioning System

**Date:** 2026-02-04  
**Status:** 90% Complete (P1-F pending)

---

## Summary

| Milestone | Description | Lines | Tests | Status |
|-----------|-------------|-------|-------|--------|
| P1-A | Anchor GNSS Tracking | 970 | 16 | Complete |
| P1-B | Inter-Anchor Fusion | 820 | 20 | Complete |
| P1-C | Tag Multilateration | 725 | 13 | Complete |
| P1-D | Robustness | 900 | 16 | Complete |
| P1-E | Gate Metrics | 500 | 15 | Complete |
| P1-F | BAH Uplink | - | - | Pending |
| **Total** | | **3,915** | **80** | **90%** |

---

## Architecture

```
Input: GNSS + Inter-Anchor UWB + Tag UWB

P1-A: Anchor Tracking
  GNSS → 6D Kalman Filter → anchor_states

P1-B: Inter-Anchor Fusion
  anchor_states + inter_anchor_ranges → refined_anchors

P1-D: Tag Range Gating
  tag_ranges → validated_ranges

P1-C: Tag Multilateration
  validated_ranges + refined_anchors → raw_estimate

P1-D: Plausibility + Kinematic Filter
  raw_estimate → smoothed_estimate

P1-E: Gate Metrics
  smoothed_estimate + anchors → gate_metrics

Output: Position + Velocity + Quality + Gate Distance + Crossings
```

---

## Performance

| Stage | Time (ms) |
|-------|-----------|
| Anchor tracking (P1-A) | 0.10 |
| Inter-anchor fusion (P1-B) | 0.21 |
| Tag range gating (P1-D) | 0.01 |
| Tag multilateration (P1-C) | 0.05 |
| Plausibility check (P1-D) | 0.01 |
| Kinematic filter (P1-D) | 0.02 |
| Gate metrics (P1-E) | < 0.01 |
| **Total** | **0.41** |

**Budget:** 100ms (10Hz) | **Margin:** 244x

---

## P1-A: Anchor GNSS Tracking

**Purpose:** Track anchor positions with GNSS using Kalman filtering.

**Modules:**
- `bah_core/localization/anchor_tracker.py`
- `bah_core/localization/gnss_quality.py`
- `bah_core/localization/anchor_state.py`
- `bah_core/metrics/counters.py`

**Features:**
- 6D Kalman filter (position + velocity)
- GNSS quality assessment
- Uncertainty propagation
- Stale fix handling

**Usage:**
```python
from bah_core.localization import AnchorTracker, GNSSQuality, GNSSFixType

tracker = AnchorTracker("A0")
quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.5, num_satellites=8)
tracker.update_gnss(time.time(), (0, 0, 2), quality)
state = tracker.predict(time.time() + 1.0)
```

---

## P1-B: Inter-Anchor Fusion

**Purpose:** Refine anchor geometry using inter-anchor UWB ranges.

**Modules:**
- `bah_core/proto/inter_anchor_range.py`
- `bah_core/localization/range_gating.py`
- `bah_core/localization/anchor_network_fusion.py`

**Features:**
- Message schema for inter-anchor ranges
- 5-level range gating
- WLS Gauss-Newton solver
- 30%+ geometry improvement
- Automatic GNSS-only fallback

**Usage:**
```python
from bah_core.localization import AnchorNetworkFusion, InterAnchorRangeGate
from bah_core.proto import InterAnchorRangeReport, RangeQuality

fusion = AnchorNetworkFusion(range_gate=InterAnchorRangeGate())
result = fusion.fuse(gnss_states, ranges, t_now)
refined_anchors = result.refined_anchors
```

---

## P1-C: Tag Multilateration

**Purpose:** Solve tag position from UWB ranges to anchors.

**Modules:**
- `bah_core/proto/tag_range.py`
- `bah_core/proto/position_estimate.py`
- `bah_core/localization/tag_solver.py`

**Features:**
- 2D Gauss-Newton solver (water surface)
- 3D Gauss-Newton solver (full)
- < 20cm accuracy (2D)
- Geometry and quality scoring
- Automatic NO_FIX handling

**Usage:**
```python
from bah_core.localization import TagSolver, TagSolverConfig
from bah_core.proto import TagRangeReport, TagRangeQuality

solver = TagSolver(TagSolverConfig(prefer_2d=True))
estimate = solver.solve(ranges, anchor_states)
```

---

## P1-D: Robustness

**Purpose:** Prevent outliers from causing position jumps; smooth output.

**Modules:**
- `bah_core/localization/tag_range_gating.py`
- `bah_core/localization/tag_kinematic_filter.py`
- `bah_core/localization/plausibility_checker.py`
- `bah_core/localization/robust_tag_positioning.py`

**Features:**
- Tag range gating (4 criteria)
- 4D constant-velocity Kalman filter
- Plausibility checking (speed, accel, innovation)
- Holdover support (2s max)
- Quality degradation signaling

**Usage:**
```python
from bah_core.localization import create_default_pipeline

pipeline = create_default_pipeline("T1")
estimate = pipeline.process(ranges, anchor_states, t_solve)
```

---

## P1-E: Gate Metrics

**Purpose:** Calculate distance to start line and detect crossings.

**Modules:**
- `bah_core/proto/gate_metrics.py`
- `bah_core/domain/gate_calculator.py`

**Features:**
- Perpendicular distance (signed)
- Along-line coordinate (0-1)
- Crossing detection with direction
- 4-factor confidence scoring

**Usage:**
```python
from bah_core.domain import create_default_gate_calculator

gate_calc = create_default_gate_calculator("A0", "A1")
gate_metrics = gate_calc.compute_metrics(position_estimate, anchor_states)
```

---

## P1-F: BAH Uplink (Pending)

**Purpose:** Transmit position and gate metrics to server.

**Planned:**
- Network client for server communication
- Message buffering and retry logic
- HealthStatus assembly

---

## Acceptance Criteria

| Milestone | Criteria | Status |
|-----------|----------|--------|
| P1-A | Continuous anchor ENU updates | Pass |
| P1-A | Predict to arbitrary time | Pass |
| P1-A | Stale GNSS inflates uncertainty | Pass |
| P1-B | Stationary anchors < 20cm | Pass |
| P1-B | GNSS jitter reduction 30%+ | Pass |
| P1-B | Clean GNSS-only fallback | Pass |
| P1-C | Stable simulated tag position | Pass |
| P1-C | NO_FIX on missing ranges | Pass |
| P1-C | Smooth with moving anchors | Pass |
| P1-D | No teleporting from outliers | Pass |
| P1-D | Smooth holdover (2s max) | Pass |
| P1-E | Stable distance (no GNSS artifacts) | Pass |
| P1-E | Consistent crossing detection | Pass |

**Result:** 13/13 criteria met (100%)

---

## Complete Usage Example

```python
from bah_core.localization import (
    AnchorTracker,
    AnchorNetworkFusion,
    create_default_pipeline,
    GNSSQuality,
    GNSSFixType,
)
from bah_core.domain import create_default_gate_calculator
from bah_core.proto import InterAnchorRangeReport, TagRangeReport, RangeQuality, TagRangeQuality

# Setup
trackers = {"A0": AnchorTracker("A0"), "A1": AnchorTracker("A1"), "A2": AnchorTracker("A2")}
fusion = AnchorNetworkFusion()
tag_pipeline = create_default_pipeline("T1")
gate_calc = create_default_gate_calculator("A0", "A1")

# Processing loop
t_now = time.time()

# 1. Update anchors with GNSS
quality = GNSSQuality(fix_type=GNSSFixType.GPS_FIX, hdop=1.5, num_satellites=8)
for aid, tracker in trackers.items():
    tracker.update_gnss(t_now, gnss_pos[aid], quality)

gnss_states = {aid: t.predict(t_now) for aid, t in trackers.items()}

# 2. Refine with inter-anchor UWB (optional)
fusion_result = fusion.fuse(gnss_states, inter_anchor_ranges, t_now)
anchor_states = fusion_result.refined_anchors

# 3. Process tag ranges
estimate = tag_pipeline.process(tag_ranges, anchor_states, t_now)

# 4. Compute gate metrics
gate_metrics = gate_calc.compute_metrics(estimate, anchor_states)

# 5. Output
if estimate.has_valid_fix:
    print(f"Position: {estimate.pos_enu}")
    print(f"Velocity: {estimate.vel_enu}")
    print(f"Gate distance: {gate_metrics.d_perp_signed:.2f}m")
    if gate_metrics.has_crossing:
        print(f"CROSSING: {gate_metrics.crossing_event.name}")
```
