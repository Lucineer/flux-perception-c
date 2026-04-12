# MAINTENANCE.md

## Architecture

- `perception.h` — Public API and data types
- `perception.c` — Implementation (links with `-lm` for `sqrtf`)
- `test_perception.c` — 21 tests covering all API surface
- `Makefile` — Builds static lib and runs tests

## Key Design Decisions

- **Ring buffer** for history: `signal_count` increments forever, index = count % SIGNALS_MAX
- **Fusion formula**: `Σ(value+bias)*weight*confidence / Σ(weight*confidence)`
- **Agreement**: `1 - stddev/mean_abs`. Clamped to [0,1]. Single sensor = 1.0, zero sensors = 0.0
- **No malloc**: All storage is inline in `PerceptionEngine`

## Limits

- `SENSORS_MAX 16` — max concurrent sensors
- `SIGNALS_MAX 64` — history ring buffer depth
- No thread safety — caller must synchronize

## Adding Features

- New fusion strategies: add alternative read functions, keep `perc_read` as default
- More history: increase `SIGNALS_MAX` (grows struct)
- Conditional compilation: gate math with `#ifdef PERCEPTION_USE_MATH`

## Testing

Run `make test`. Each test is self-contained with its own engine instance.
