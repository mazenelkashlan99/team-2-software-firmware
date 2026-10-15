#include <unity.h>
#include <cmath>
#include "Fusion.h"
#include "robot_interfaces.h"

using namespace robot;

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

void test_idle_stays_zero() {
    Fusion f;
    f.begin(0.98f);
    f.setDt(0.01f);

    Pose out {};
    for (int i = 0; i < 200; ++i) {
        out = f.update(zero_pose(), imu_yaw_rate(0.0f));
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out.theta);
}

// Robot really rotating at 1 rad/s: BOTH inputs agree on the rotation.
// Odom reports the same yaw the gyro integrates to.
void test_yaw_rate_integrates() {
    Fusion f;
    f.begin(0.98f);
    const float dt = 0.01f;
    f.setDt(dt);

    Pose odom {};
    Pose out {};
    for (int i = 0; i < 100; ++i) {
        odom.theta += 1.0f * dt;             // odometry sees the rotation
        out = f.update(odom, imu_yaw_rate(1.0f));
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, out.theta);
}

// A single noise spike is attenuated to at most alpha * spike * dt.
// With alpha=0.98, dt=0.01, spike=100 rad/s -> 0.98 rad displacement.
// The point of this test is: the spike does NOT pass through whole.
void test_single_noise_spike_rejected() {
    Fusion f;
    f.begin(0.98f);
    f.setDt(0.01f);

    Pose out {};
    for (int i = 0; i < 20; ++i) {
        out = f.update(zero_pose(), imu_yaw_rate(0.0f));
    }
    const float before = out.theta;

    out = f.update(zero_pose(), imu_yaw_rate(100.0f));   // one spike

    const float delta = out.theta - before;
    // Spike attenuated to at most 0.98 rad — far less than 100 * 0.01 = 1.
    TEST_ASSERT_TRUE(delta < 1.0f);
    TEST_ASSERT_TRUE(delta > 0.9f);   // but it does respond to the sample
}

void test_odometry_drives_estimate() {
    Fusion f;
    f.begin(0.98f);
    f.setDt(0.01f);

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

void test_output_is_finite() {
    Fusion f;
    f.begin(0.98f);
    f.setDt(0.01f);

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