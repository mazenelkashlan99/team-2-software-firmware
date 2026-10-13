#include <unity.h>
#include "PID.h"

// Helpers ------------------------------------------------------------

static void settle(PID& p, float sp, float meas, float dt, int n) {
    for (int i = 0; i < n; ++i) p.compute(sp, meas, dt);
}

// setUp / tearDown are required by Unity.

void setUp() {}
void tearDown() {}

// Tests --------------------------------------------------------------

// FF alone: kp=ki=kd=0, kff=2, sp=0.3 -> output 0.6.
void test_feedforward_only() {
    PID p;
    p.setGains(0.0f, 0.0f, 0.0f, 2.0f);
    const float out = p.compute(0.3f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.6f, out);
}

// Output never exceeds outMax_ even for a huge setpoint.
void test_output_clamped() {
    PID p;
    p.setGains(10.0f, 0.0f, 0.0f, 5.0f);
    p.setOutputLimits(-1.0f, 1.0f);
    const float out = p.compute(100.0f, 0.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out);
}

// dt <= 0 must not poison state; later calls with valid dt still work.
void test_dt_zero_is_safe() {
    PID p;
    p.setGains(1.0f, 1.0f, 0.0f, 0.0f);

    const float bad = p.compute(1.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(isfinite(bad));
    TEST_ASSERT_FALSE(isnan(bad));

    // A normal call afterwards must still produce a sane value.
    const float good = p.compute(1.0f, 0.0f, 0.01f);
    TEST_ASSERT_TRUE(isfinite(good));
    TEST_ASSERT_FALSE(isnan(good));
}

// Anti-windup: after long saturation, integral must not stay pinned
// so long that a reversed error takes hundreds of ticks to leave the limit.
void test_no_windup_after_saturation() {
    PID p;
    p.setGains(0.0f, 1.0f, 0.0f, 0.0f);
    p.setOutputLimits(-1.0f, 1.0f);
    p.setIntegralLimits(-1.0f, 1.0f);

    // Hold a large positive error for 1000 ticks — output saturates.
    for (int i = 0; i < 1000; ++i) p.compute(10.0f, 0.0f, 0.01f);

    // Reverse the error. Output must leave the positive limit within
    // a few ticks, not hundreds.
    int ticksToLeave = -1;
    for (int i = 0; i < 50; ++i) {
        const float out = p.compute(-1.0f, 0.0f, 0.01f);
        if (out < 1.0f - 1e-3f) { ticksToLeave = i; break; }
    }
    TEST_ASSERT_TRUE_MESSAGE(ticksToLeave >= 0 && ticksToLeave < 10,
                             "integral stayed saturated too long");
}

// Derivative on measurement: a setpoint step with measurement constant
// must not produce a spike in the output.
void test_no_derivative_kick_on_setpoint_step() {
    PID p;
    p.setGains(0.0f, 0.0f, 1.0f, 0.0f);

    // Warm up: settle at sp=meas=0.
    for (int i = 0; i < 5; ++i) p.compute(0.0f, 0.0f, 0.01f);

    // Step the setpoint. Measurement unchanged.
    const float out = p.compute(1.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out);
}

// reset() forgets history.
void test_reset_clears_history() {
    PID p;
    p.setGains(0.0f, 1.0f, 0.0f, 0.0f);

    for (int i = 0; i < 10; ++i) p.compute(1.0f, 0.0f, 0.01f);
    const float before = p.compute(1.0f, 0.0f, 0.01f);
    TEST_ASSERT_TRUE(before != 0.0f);

    p.reset();
    const float after = p.compute(0.0f, 0.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, after);
}

// -- Runner ----------------------------------------------------------

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_feedforward_only);
    RUN_TEST(test_output_clamped);
    RUN_TEST(test_dt_zero_is_safe);
    RUN_TEST(test_no_windup_after_saturation);
    RUN_TEST(test_no_derivative_kick_on_setpoint_step);
    RUN_TEST(test_reset_clears_history);
    return UNITY_END();
}