#ifndef PERCEPTION_H
#define PERCEPTION_H
#include <stdint.h>
#include <stddef.h>

#define SENSORS_MAX 16
#define SIGNALS_MAX 64

typedef struct {
    uint8_t sensor_id;
    float value;        // raw reading
    float confidence;   // 0.0-1.0 reliability
    float weight;       // blend weight
    float bias;         // calibration offset
    uint64_t timestamp;
    uint8_t active;
} Sensor;

typedef struct {
    float value;        // fused result
    float confidence;   // fused confidence
    float variance;     // agreement measure
    uint8_t source_count;
    uint64_t timestamp;
} FusedSignal;

typedef struct {
    Sensor sensors[SENSORS_MAX];
    uint8_t sensor_count;
    FusedSignal signals[SIGNALS_MAX];
    uint16_t signal_count;
    float threshold;    // minimum confidence to accept
} PerceptionEngine;

void perc_init(PerceptionEngine *e, float threshold);
int perc_add_sensor(PerceptionEngine *e, uint8_t id, float weight, float bias);
Sensor* perc_find_sensor(PerceptionEngine *e, uint8_t id);
int perc_update(PerceptionEngine *e, uint8_t sensor_id, float value, float confidence, uint64_t now);
FusedSignal perc_read(const PerceptionEngine *e);
int perc_history(const PerceptionEngine *e, FusedSignal *buf, int max);
float perc_agreement(const PerceptionEngine *e);
int perc_deactivate(PerceptionEngine *e, uint8_t sensor_id);
int perc_calibrate(PerceptionEngine *e, uint8_t sensor_id, float bias);

#endif
