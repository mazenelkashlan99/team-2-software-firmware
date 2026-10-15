#include <unity.h>
#include "RateLimiter.h"

void setUp() {}
void tearDown() {}

// A rate-limited value must not move faster than maxRate * dt.
void test_respects_rate() {
    RateLimiter r;
    r.setMaxRate(1.0f);     // 1 unit/s
    r.reset(0.0f);

    // Target 10, dt 0.1 -> at most 0.1 of movement.
    const float v = r.update(10.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, v);
}

// Multiple small steps eventually reach the target.
void test_reaches_target_eventually() {
    RateLimiter r;
    r.setMaxRate(1.0f);
    r.reset(0.0f);

    float v = 0.0f;
    for (int i = 0; i < 20; ++i) v = r.update(1.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, v);
}

// When the target is closer than one step, snap exactly to it (no
// overshoot, no oscillation).
void test_snaps_when_within_one_step() {
    RateLimiter r;
    r.setMaxRate(1.0f);
    r.reset(0.0f);

    const float v = r.update(0.05f, 0.1f);   // max step 0.1, target 0.05
    TEST_ASSERT_EQUAL_FLOAT(0.05f, v);
}

// maxRate == 0 means "no limit, follow target immediately".
void test_zero_rate_snaps() {
    RateLimiter r;
    r.setMaxRate(0.0f);
    r.reset(0.0f);

    const float v = r.update(42.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, v);
}

// dt <= 0 must not change anything.
void test_dt_zero_no_change() {
    RateLimiter r;
    r.setMaxRate(1.0f);
    r.reset(0.5f);

    const float v = r.update(10.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, v);
}

// Negative rate is ignored (0 stays as set, previous value kept).
void test_negative_rate_ignored() {
    RateLimiter r;
    r.setMaxRate(2.0f);
    r.setMaxRate(-5.0f);   // ignored, 2.0 stays

    r.reset(0.0f);
    const float v = r.update(10.0f, 0.1f);   // expect step = 0.2
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, v);
}

// reset() must not change the rate setting.
void test_reset_keeps_rate() {
    RateLimiter r;
    r.setMaxRate(1.0f);
    r.reset(0.0f);
    r.reset(5.0f);

    const float v = r.update(100.0f, 0.1f);  // step = 0.1
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.1f, v);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_respects_rate);
    RUN_TEST(test_reaches_target_eventually);
    RUN_TEST(test_snaps_when_within_one_step);
    RUN_TEST(test_zero_rate_snaps);
    RUN_TEST(test_dt_zero_no_change);
    RUN_TEST(test_negative_rate_ignored);
    RUN_TEST(test_reset_keeps_rate);
    return UNITY_END();
}