# flux-perception-c 👁️

**C11 sensor fusion engine for edge agents.** Confidence-weighted blending of up to 16 sensors into a single fused signal. Designed for embedded deployment on ESP32, Jetson, or CUDA.

```c
PerceptionEngine e;
perc_init(&e, 0.3f);  // minimum confidence threshold

perc_add_sensor(&e, /* id: */ 1, /* weight: */ 0.5, /* bias: */ 0.0);
perc_add_sensor(&e, 2, 0.3, 0.0);
perc_add_sensor(&e, 3, 0.2, 0.0);

perc_update(&e, 1, 48.5, 0.95, now);  // high-confidence reading
perc_update(&e, 2, 52.0, 0.60, now);  // low-confidence
perc_update(&e, 3, 45.0, 0.80, now);

FusedSignal fs = perc_read(&e);
printf("Fused: %.2f (confidence: %.2f, variance: %.2f)\n",
       fs.value, fs.confidence, fs.variance);
```

## API

```c
// Initialize with confidence threshold
PerceptionEngine e;
perc_init(&e, 0.3f);

// Register sensors
perc_add_sensor(&e, 1, 0.5, 0.0);  // id, blend weight, bias

// Update sensor readings
perc_update(&e, 1, 48.5, 0.95, now);  // id, value, confidence, timestamp

// Fuse all active sensors into one signal
FusedSignal fs = perc_read(&e);
// fs.value — weighted blend
// fs.confidence — agreement-weighted
// fs.variance — disagreement measure

// Get sensor by ID
Sensor* s = perc_find_sensor(&e, 1);

// Read history
FusedSignal history[10];
int n = perc_history(&e, history, 10);
```

### Fusion Algorithm

1. Filter sensors below confidence threshold
2. Weight each by `weight × confidence` (bias-corrected)
3. Fused value = normalized weighted sum
4. Fused confidence = mean confidence weighted by agreement
5. Variance = disagreement signal (high = conflicting sensors)

## Build & Test

```bash
make test
```

## Why C11?

- **Embedded targets** — fits in 2KB binary on ESP32
- **Zero dependencies** — no libc beyond stdint/stddef
- **Deterministic** — no allocation, no dynamic dispatch
- **CUDA-ready** — compile with `nvcc` for GPU-parallelized sensor fusion

## Fleet Context

Part of the Lucineer/Cocapn fleet. C11 sibling to [flux-perception](https://github.com/Lucineer/flux-perception) (Rust). Pairs with [flux-confidence](https://github.com/Lucineer/flux-confidence) for calibrated uncertainty.
