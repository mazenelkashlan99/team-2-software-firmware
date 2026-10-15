#include <unity.h>
#include <cmath>
#include "PID.h"

static float motor_step(float v, float u, float K, float tau, float dt) {
    return v + ((K * u - v) / tau) * dt;
}

void setUp() {}
void tearDown() {}

void test_converges_to_setpoint() {
    const float K = 1.2f, tau = 0.08f, dt = 0.01f, sp = 0.3f;
    PID pid;
    pid.setGains(0.5f, 4.0f, 0.0f, 1.0f / K);
    pid.setOutputLimits(-1.0f, 1.0f);
    pid.setIntegralLimits(-0.5f, 0.5f);

    float v = 0.0f;
    for (int i = 0; i < 200; ++i) {
        const float u = pid.compute(sp, v, dt);
        v = motor_step(v, u, K, tau, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.006f, sp, v);
}

void test_settles_within_500ms() {
    const float K = 1.2f, tau = 0.08f, dt = 0.01f, sp = 0.3f;
    PID pid;
    pid.setGains(0.5f, 4.0f, 0.0f, 1.0f / K);
    pid.setOutputLimits(-1.0f, 1.0f);
    pid.setIntegralLimits(-0.5f, 0.5f);

    float v = 0.0f;
    int settleTick = -1;
    const float band = 0.05f * sp;
    for (int i = 0; i < 200; ++i) {
        const float u = pid.compute(sp, v, dt);
        v = motor_step(v, u, K, tau, dt);
        if (settleTick < 0 && std::fabs(v - sp) < band) {
            bool stays = true;
            float vv = v;
            for (int j = i + 1; j < 200 && stays; ++j) {
                const float uu = pid.compute(sp, vv, dt);
                vv = motor_step(vv, uu, K, tau, dt);
                if (std::fabs(vv - sp) >= band) stays = false;
            }
            if (stays) settleTick = i;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(settleTick >= 0 && settleTick < 50,
                             "did not settle within 500 ms");
}

void test_output_clamped_when_stalled() {
    const float dt = 0.01f;
    PID pid;
    pid.setGains(1.0f, 10.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1.0f, 1.0f);
    pid.setIntegralLimits(-0.5f, 0.5f);
    for (int i = 0; i < 1000; ++i) {
        const float u = pid.compute(0.5f, 0.0f, dt);
        TEST_ASSERT_TRUE(u <= 1.0f);
        TEST_ASSERT_TRUE(u >= -1.0f);
    }
}

void test_no_windup_after_stall() {
    const float dt = 0.01f;
    PID pid;
    pid.setGains(1.0f, 10.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1.0f, 1.0f);
    pid.setIntegralLimits(-0.5f, 0.5f);
    for (int i = 0; i < 1000; ++i) pid.compute(0.5f, 0.0f, dt);
    const float out = pid.compute(-0.5f, 0.0f, dt);
    TEST_ASSERT_TRUE_MESSAGE(out < 0.0f, "integral stayed wound up");
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_converges_to_setpoint);
    RUN_TEST(test_settles_within_500ms);
    RUN_TEST(test_output_clamped_when_stalled);
    RUN_TEST(test_no_windup_after_stall);
    return UNITY_END();
}