#include <unity.h>
#include <cmath>
#include "Odometry.h"

// TODO: confirm wheelbase and encoder calibration match your config.
static constexpr float TRACK_W = 0.15f;   // meters
static constexpr float TOL     = 1e-3f;

void setUp() {}
void tearDown() {}

// Straight-line motion: equal deltas on both wheels -> x advances,
// y and theta stay zero.
void test_straight_forward() {
    Odometry o;
    o.begin(TRACK_W);

    for (int i = 0; i < 10; ++i) o.update(0.01f, 0.01f);   // 10 * 1 cm each

    const Pose p = o.pose();   // TODO: adapt to your real type
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.10f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.00f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.00f, p.theta);
}

// Pure rotation in place: equal and opposite deltas. Robot spins,
// but x and y don't move.
void test_spin_in_place() {
    Odometry o;
    o.begin(TRACK_W);

    // dL = -1 cm, dR = +1 cm per tick
    for (int i = 0; i < 10; ++i) o.update(-0.01f, 0.01f);

    const Pose p = o.pose();
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, p.y);
    // theta = sum(dR - dL) / trackWidth per tick = 10 * 0.02 / 0.15
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 10.0f * 0.02f / TRACK_W, p.theta);
}

// Forward + turn: expected x, y, theta after a known command.
// Rather than gold-plating this, sanity-check the invariants:
// distance traveled must equal wheel-average * ticks.
void test_arc_path_consistent() {
    Odometry o;
    o.begin(TRACK_W);

    const int N = 100;
    const float dL = 0.01f;
    const float dR = 0.012f;   // slightly faster right wheel -> left turn
    for (int i = 0; i < N; ++i) o.update(dL, dR);

    const Pose p = o.pose();
    const float distMoved = sqrtf(p.x * p.x + p.y * p.y);
    TEST_ASSERT_TRUE(distMoved > 0.0f);
    TEST_ASSERT_TRUE(p.theta > 0.0f);   // CCW when right wheel is faster
}

// resetPose() zeroes x, y, theta.
void test_reset_pose() {
    Odometry o;
    o.begin(TRACK_W);

    for (int i = 0; i < 10; ++i) o.update(0.01f, 0.01f);
    o.resetPose(0.0f, 0.0f, 0.0f);   // TODO: name may differ

    const Pose p = o.pose();
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, p.theta);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_forward);
    RUN_TEST(test_spin_in_place);
    RUN_TEST(test_arc_path_consistent);
    RUN_TEST(test_reset_pose);
    return UNITY_END();
}