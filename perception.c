#include "perception.h"
#include <math.h>

void perc_init(PerceptionEngine *e, float threshold) {
    for (int i = 0; i < SENSORS_MAX; i++) {
        e->sensors[i].active = 0;
    }
    for (int i = 0; i < SIGNALS_MAX; i++) {
        e->signals[i].value = 0;
        e->signals[i].confidence = 0;
        e->signals[i].variance = 0;
        e->signals[i].source_count = 0;
        e->signals[i].timestamp = 0;
    }
    e->sensor_count = 0;
    e->signal_count = 0;
    e->threshold = threshold;
}

int perc_add_sensor(PerceptionEngine *e, uint8_t id, float weight, float bias) {
    if (e->sensor_count >= SENSORS_MAX) return -1;
    for (int i = 0; i < e->sensor_count; i++) {
        if (e->sensors[i].sensor_id == id) return -1;
    }
    Sensor *s = &e->sensors[e->sensor_count++];
    s->sensor_id = id;
    s->value = 0;
    s->confidence = 0;
    s->weight = weight;
    s->bias = bias;
    s->timestamp = 0;
    s->active = 1;
    return 0;
}

Sensor* perc_find_sensor(PerceptionEngine *e, uint8_t id) {
    for (int i = 0; i < e->sensor_count; i++) {
        if (e->sensors[i].sensor_id == id) return &e->sensors[i];
    }
    return NULL;
}

int perc_update(PerceptionEngine *e, uint8_t sensor_id, float value, float confidence, uint64_t now) {
    Sensor *s = perc_find_sensor(e, sensor_id);
    if (!s) return -1;
    s->value = value;
    s->confidence = confidence;
    s->timestamp = now;
    s->active = 1;

    // Record fused signal into ring buffer
    FusedSignal fused = perc_read(e);
    uint16_t idx = e->signal_count % SIGNALS_MAX;
    e->signals[idx] = fused;
    e->signal_count++;
    return 0;
}

FusedSignal perc_read(const PerceptionEngine *e) {
    FusedSignal out = {0, 0, 0, 0, 0};
    double wsum = 0, vsum = 0, csum = 0;
    int count = 0;
    uint64_t latest = 0;

    for (int i = 0; i < e->sensor_count; i++) {
        const Sensor *s = &e->sensors[i];
        if (!s->active || s->confidence < e->threshold) continue;
        float w = s->weight * s->confidence;
        float corrected = s->value + s->bias;
        wsum += w;
        vsum += (double)corrected * w;
        csum += s->confidence;
        if (s->timestamp > latest) latest = s->timestamp;
        count++;
    }

    if (wsum > 0) {
        out.value = (float)(vsum / wsum);
        out.confidence = (float)(csum / count);
    }
    out.source_count = (uint8_t)count;
    out.variance = perc_agreement(e);
    out.timestamp = latest;
    return out;
}

int perc_history(const PerceptionEngine *e, FusedSignal *buf, int max) {
    int total = e->signal_count < SIGNALS_MAX ? e->signal_count : SIGNALS_MAX;
    int n = total < max ? total : max;
    for (int i = 0; i < n; i++) {
        // most recent first
        uint16_t idx = (e->signal_count - 1 - i) % SIGNALS_MAX;
        buf[i] = e->signals[idx];
    }
    return n;
}

float perc_agreement(const PerceptionEngine *e) {
    float mean = 0;
    int count = 0;
    for (int i = 0; i < e->sensor_count; i++) {
        const Sensor *s = &e->sensors[i];
        if (!s->active || s->confidence < e->threshold) continue;
        mean += s->value + s->bias;
        count++;
    }
    if (count < 2) return count == 1 ? 1.0f : 0.0f;
    mean /= count;

    float max_abs = 0;
    double variance = 0;
    for (int i = 0; i < e->sensor_count; i++) {
        const Sensor *s = &e->sensors[i];
        if (!s->active || s->confidence < e->threshold) continue;
        float corrected = s->value + s->bias;
        float diff = corrected - mean;
        variance += (double)diff * diff;
        float abs_val = corrected < 0 ? -corrected : corrected;
        if (abs_val > max_abs) max_abs = abs_val;
    }
    variance /= count;
    float stddev = sqrtf(variance);

    if (max_abs < 1e-9f) return 1.0f;
    float ratio = stddev / max_abs;
    if (ratio > 1.0f) ratio = 1.0f;
    return 1.0f - ratio;
}

int perc_deactivate(PerceptionEngine *e, uint8_t sensor_id) {
    Sensor *s = perc_find_sensor(e, sensor_id);
    if (!s) return -1;
    s->active = 0;
    return 0;
}

int perc_calibrate(PerceptionEngine *e, uint8_t sensor_id, float bias) {
    Sensor *s = perc_find_sensor(e, sensor_id);
    if (!s) return -1;
    s->bias = bias;
    return 0;
}
