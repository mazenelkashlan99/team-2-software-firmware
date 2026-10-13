#include <unity.h>
#include <cmath>
#include "Fusion.h"
#include "robot_interfaces.h"

using namespace robot;

// Synthesize an IMU sample. Only gz is used by a yaw-only filter.
static ImuData imu_yaw_rate(float gz_rad_per_s) {
    ImuData d {};
    d.gz = gz_rad_per_s;
    return d;
}

static Pose zero_pose() {
    Pose p {};
    return p;
}

void setUp() {}
void tearDown() {}

// No IMU motion, no odometry -> estimate stays put.
void test_idle_stays_zero() {
    Fusion f;
    f.begin(0.98f);

    Pose out {};
    for (int i = 0; i < 200; ++i) {
        out = f.update(zero_pose(), imu_yaw_rate(0.0f));
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out.theta);
}

// A constant yaw rate of 1 rad/s for 1 s must produce theta ~ 1 rad.
void test_yaw_rate_integrates() {
    Fusion f;
    f.begin(0.98f);

    Pose out {};
    const float dt = 0.01f;
    f.setDt(dt);   // adjust name; some filters take dt per update() instead
    for (int i = 0; i < 100; ++i) {   // 1 second
        out = f.update(zero_pose(), imu_yaw_rate(1.0f));
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, out.theta);
}

// A single noisy sample must not move the estimate much.
void test_single_noise_spike_rejected() {
    Fusion f;
    f.begin(0.98f);

    Pose out {};
    for (int i = 0; i < 20; ++i) {
        out = f.update(zero_pose(), imu_yaw_rate(0.0f));
    }
    const float before = out.theta;

    out = f.update(zero_pose(), imu_yaw_rate(100.0f));   // huge spike

    TEST_ASSERT_FLOAT_WITHIN(0.01f, before, out.theta);
}

// Odometry alone must move the estimate (zero IMU).
void test_odometry_drives_estimate() {
    Fusion f;
    f.begin(0.98f);

    Pose odom {};
    odom.x     = 0.1f;
    odom.y     = 0.0f;
    odom.theta = 0.05f;

    Pose out {};
    for (int i = 0; i < 50; ++i) {
        out = f.update(odom, imu_yaw_rate(0.0f));
    }
    TEST_ASSERT_TRUE(out.x > 0.0f);
}

// Fused output must be finite (no NaN from a first-call bug or divide).
void test_output_is_finite() {
    Fusion f;
    f.begin(0.98f);

    Pose out {};
    for (int i = 0; i < 10; ++i) {
        out = f.update(zero_pose(), imu_yaw_rate(0.5f));
        TEST_ASSERT_TRUE(std::isfinite(out.x));
        TEST_ASSERT_TRUE(std::isfinite(out.y));
        TEST_ASSERT_TRUE(std::isfinite(out.theta));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_idle_stays_zero);
    RUN_TEST(test_yaw_rate_integrates);
    RUN_TEST(test_single_noise_spike_rejected);
    RUN_TEST(test_odometry_drives_estimate);
    RUN_TEST(test_output_is_finite);
    return UNITY_END();
}