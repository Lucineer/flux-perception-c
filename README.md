# flux-perception-c

Zero-dependency, no-malloc sensor fusion library in pure C11.

Confidence-weighted blending of multiple signal sources with calibration offsets, agreement metrics, and a ring-buffer history.

## Features

- Confidence-weighted average fusion
- Per-sensor weight and bias calibration
- Agreement metric (0.0–1.0)
- Ring-buffer history (up to 64 fused signals)
- Threshold filtering (ignore low-confidence sensors)
- Zero dynamic allocation

## Build

```sh
make          # builds libperception.a and runs tests
make clean
```

## Usage

```c
#include "perception.h"

PerceptionEngine e;
perc_init(&e, 0.1f);  // 0.1 minimum confidence

perc_add_sensor(&e, 1, 1.0f, 0.0f);   // id, weight, bias
perc_add_sensor(&e, 2, 0.5f, -2.0f);

perc_update(&e, 1, 22.5f, 0.9f, now);
perc_update(&e, 2, 25.0f, 0.7f, now);

FusedSignal f = perc_read(&e);
printf("fused: %.2f (confidence: %.2f)\n", f.value, f.confidence);
```

## API

| Function | Description |
|---|---|
| `perc_init` | Initialize engine with confidence threshold |
| `perc_add_sensor` | Register a sensor (id, weight, bias) |
| `perc_find_sensor` | Lookup sensor by id |
| `perc_update` | Feed a new reading |
| `perc_read` | Get current fused signal |
| `perc_history` | Retrieve last N fused signals |
| `perc_agreement` | 0.0–1.0 agreement between active sensors |
| `perc_deactivate` | Disable a sensor |
| `perc_calibrate` | Set sensor bias offset |

## License

Public domain / MIT
