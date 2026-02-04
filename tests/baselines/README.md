# Performance Baselines

This directory stores performance baseline metrics for regression testing.

## Purpose

Baselines capture the algorithm's accuracy before refactoring. After changes,
compare against these baselines to detect improvements or regressions.

## Files

- `performance_baseline.json` - 2D trilateration metrics
- `performance_baseline_3d.json` - 3D trilateration metrics
- `performance_baseline_noise.json` - Noise robustness metrics (±5cm)

## Usage

### Record Baseline (Before Changes)

```bash
# Record all baseline metrics
pytest tests/test_baseline_record.py -v --baseline-record
```

### Compare After Changes

```bash
# Compare current performance against baseline
pytest tests/test_baseline_compare.py -v
```

### Quick Check (No Baseline Required)

```bash
# Verify current algorithm meets absolute thresholds
pytest tests/test_baseline_compare.py::TestThresholdCompliance -v
```

## Metrics Captured

Each baseline records:

| Metric | Description |
|--------|-------------|
| `rmse_m` | Root mean square error (meters) |
| `max_error_m` | Maximum error observed |
| `percentile_95_m` | 95th percentile error |
| `mean_error_m` | Mean error across test cases |
| `std_error_m` | Standard deviation of errors |
| `success_rate` | Fraction of successful solves |
| `mean_solve_time_ms` | Average computation time |

## Interpreting Results

### Comparison Output

```
IMPROVEMENTS:
  - rmse_m:          0.0850 → 0.0720 (-15.29%) [IMPROVEMENT]

DEGRADATIONS:
  - max_error_m:     0.3200 → 0.4100 (+28.13%) [DEGRADATION]

UNCHANGED:
  - success_rate:    0.9800 → 0.9750 (-0.51%) [NEUTRAL]
```

### Verdict Rules

- **IMPROVEMENT**: Metric improved by >5%
- **DEGRADATION**: Metric worsened by >5%
- **NEUTRAL**: Change within ±5% (not significant)

## Git Workflow

1. **Before refactoring:** Run baseline recording and commit baselines
2. **After changes:** Run comparison tests
3. **If degradation:** Fix or document acceptable trade-offs
4. **If improvement:** Update baseline and commit

## Thresholds

See `tests/conftest.py::PERFORMANCE_THRESHOLDS` for acceptance criteria:

- Max RMSE: 0.15m
- Max 95th percentile: 0.25m
- Max error: 0.50m
- Min success rate: 98%
