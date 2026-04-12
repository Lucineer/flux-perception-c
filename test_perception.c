#include "perception.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) printf("  TEST: %-50s ", name);
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)
#define ASSERT_NEAR(a, b, eps) do { float _a=(a),_b=(b),_e=(eps); if(fabsf(_a-_b)<=_e){}else{printf("FAIL: %f != %f (eps=%f)\n",_a,_b,_e);tests_failed++;return;} } while(0)
#define ASSERT_EQ(a, b) do { if((a)==(b)){}else{printf("FAIL: %d != %d\n",(int)(a),(int)(b));tests_failed++;return;} } while(0)

void test_init(void) {
    PerceptionEngine e;
    perc_init(&e, 0.1f);
    ASSERT_EQ(e.sensor_count, 0);
    ASSERT_EQ(e.signal_count, 0);
    ASSERT_NEAR(e.threshold, 0.1f, 1e-6f);
    PASS();
}

void test_add_sensor(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    ASSERT_EQ(perc_add_sensor(&e, 1, 1.0f, 0.0f), 0);
    ASSERT_EQ(e.sensor_count, 1);
    // duplicate
    ASSERT_EQ(perc_add_sensor(&e, 1, 1.0f, 0.0f), -1);
    PASS();
}

void test_find_sensor(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 42, 1.0f, 0.0f);
    Sensor *s = perc_find_sensor(&e, 42);
    ASSERT_EQ(s != NULL, 1);
    ASSERT_EQ(s->sensor_id, 42);
    PASS();
}

void test_find_missing(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    ASSERT_EQ(perc_find_sensor(&e, 99) == NULL, 1);
    PASS();
}

void test_update_and_read(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_update(&e, 1, 25.0f, 1.0f, 100);
    FusedSignal f = perc_read(&e);
    ASSERT_NEAR(f.value, 25.0f, 1e-5f);
    ASSERT_NEAR(f.confidence, 1.0f, 1e-5f);
    ASSERT_EQ(f.source_count, 1);
    PASS();
}

void test_confidence_weighted_fusion(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_add_sensor(&e, 2, 1.0f, 0.0f);
    perc_update(&e, 1, 10.0f, 0.8f, 100);
    perc_update(&e, 2, 30.0f, 0.2f, 100);
    // weighted: (10*0.8 + 30*0.2) / (0.8+0.2) = 14
    FusedSignal f = perc_read(&e);
    ASSERT_NEAR(f.value, 14.0f, 1e-4f);
    PASS();
}

void test_weight_blending(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 2.0f, 0.0f); // weight=2
    perc_add_sensor(&e, 2, 1.0f, 0.0f); // weight=1
    perc_update(&e, 1, 0.0f, 1.0f, 100);
    perc_update(&e, 2, 30.0f, 1.0f, 100);
    // (0*2*1 + 30*1*1) / (2*1 + 1*1) = 30/3 = 10
    FusedSignal f = perc_read(&e);
    ASSERT_NEAR(f.value, 10.0f, 1e-4f);
    PASS();
}

void test_bias_calibration(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 5.0f);
    perc_update(&e, 1, 20.0f, 1.0f, 100);
    FusedSignal f = perc_read(&e);
    ASSERT_NEAR(f.value, 25.0f, 1e-4f);
    PASS();
}

void test_calibrate_after_add(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_calibrate(&e, 1, -3.0f);
    perc_update(&e, 1, 10.0f, 1.0f, 100);
    FusedSignal f = perc_read(&e);
    ASSERT_NEAR(f.value, 7.0f, 1e-4f);
    PASS();
}

void test_threshold_filtering(void) {
    PerceptionEngine e;
    perc_init(&e, 0.5f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_add_sensor(&e, 2, 1.0f, 0.0f);
    perc_update(&e, 1, 10.0f, 0.3f, 100); // below threshold
    perc_update(&e, 2, 20.0f, 0.8f, 100);
    FusedSignal f = perc_read(&e);
    ASSERT_EQ(f.source_count, 1);
    ASSERT_NEAR(f.value, 20.0f, 1e-4f);
    PASS();
}

void test_deactivate(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_add_sensor(&e, 2, 1.0f, 0.0f);
    perc_update(&e, 1, 10.0f, 1.0f, 100);
    perc_update(&e, 2, 20.0f, 1.0f, 100);
    perc_deactivate(&e, 1);
    FusedSignal f = perc_read(&e);
    ASSERT_EQ(f.source_count, 1);
    ASSERT_NEAR(f.value, 20.0f, 1e-4f);
    PASS();
}

void test_agreement_perfect(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_add_sensor(&e, 2, 1.0f, 0.0f);
    perc_update(&e, 1, 50.0f, 1.0f, 100);
    perc_update(&e, 2, 50.0f, 1.0f, 100);
    ASSERT_NEAR(perc_agreement(&e), 1.0f, 1e-5f);
    PASS();
}

void test_agreement_poor(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_add_sensor(&e, 2, 1.0f, 0.0f);
    perc_update(&e, 1, 0.0f, 1.0f, 100);
    perc_update(&e, 2, 100.0f, 1.0f, 100);
    // mean=50, stddev=50, max_abs=100, agreement=1-50/100=0.5
    float a = perc_agreement(&e);
    ASSERT_NEAR(a, 0.5f, 0.01f);
    PASS();
}

void test_agreement_single_sensor(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    perc_update(&e, 1, 42.0f, 1.0f, 100);
    ASSERT_NEAR(perc_agreement(&e), 1.0f, 1e-5f);
    PASS();
}

void test_agreement_no_sensors(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    ASSERT_NEAR(perc_agreement(&e), 0.0f, 1e-5f);
    PASS();
}

void test_history_ring_buffer(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    for (int i = 0; i < 70; i++) {
        perc_update(&e, 1, (float)i, 1.0f, (uint64_t)i);
    }
    FusedSignal buf[5];
    int n = perc_history(&e, buf, 5);
    ASSERT_EQ(n, 5);
    // most recent first should be value 69
    ASSERT_NEAR(buf[0].value, 69.0f, 1e-4f);
    ASSERT_NEAR(buf[4].value, 65.0f, 1e-4f);
    PASS();
}

void test_history_empty(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    FusedSignal buf[3];
    ASSERT_EQ(perc_history(&e, buf, 3), 0);
    PASS();
}

void test_calibrate_missing(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    ASSERT_EQ(perc_calibrate(&e, 99, 1.0f), -1);
    PASS();
}

void test_deactivate_missing(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    ASSERT_EQ(perc_deactivate(&e, 99), -1);
    PASS();
}

void test_max_sensors(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    for (int i = 0; i < SENSORS_MAX; i++) {
        ASSERT_EQ(perc_add_sensor(&e, (uint8_t)i, 1.0f, 0.0f), 0);
    }
    ASSERT_EQ(perc_add_sensor(&e, 255, 1.0f, 0.0f), -1);
    PASS();
}

void test_read_no_active(void) {
    PerceptionEngine e;
    perc_init(&e, 0.0f);
    perc_add_sensor(&e, 1, 1.0f, 0.0f);
    // confidence=0, threshold=0, so 0 >= 0 passes the threshold check
    // but weight*confidence = 0, so sensor contributes nothing to fusion
    // count will be 1 since it passes threshold, but wsum=0 so value stays 0
    FusedSignal f = perc_read(&e);
    ASSERT_EQ(f.source_count, 1);
    PASS();
}

int main(void) {
    printf("=== flux-perception-c test suite ===\n\n");
    test_init();
    test_add_sensor();
    test_find_sensor();
    test_find_missing();
    test_update_and_read();
    test_confidence_weighted_fusion();
    test_weight_blending();
    test_bias_calibration();
    test_calibrate_after_add();
    test_threshold_filtering();
    test_deactivate();
    test_agreement_perfect();
    test_agreement_poor();
    test_agreement_single_sensor();
    test_agreement_no_sensors();
    test_history_ring_buffer();
    test_history_empty();
    test_calibrate_missing();
    test_deactivate_missing();
    test_max_sensors();
    test_read_no_active();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
