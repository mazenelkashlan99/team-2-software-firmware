#pragma once
// ---- Network identity ----------------------------------------------
#define ROBOT_SID "robot-01"

// =====================================================================
// config.h — the single hardware interface file.
//
// Everything a driver needs to know about pins, calibration, and limits
// lives here. Nothing else in the firmware hardcodes a pin number or a
// physical constant.
//
// PLACEHOLDER VALUES. The hardware team must confirm every pin number
// and every calibration constant against the actual robot before the
// first floor test. Safe defaults are fine for a bench bring-up.
// =====================================================================


// ---- Left motor driver (LEDC, dual-PWM) -----------------------------
// A DC motor driver has two direction inputs (IN1, IN2) and one enable
// per channel. Each IN pin needs its own LEDC channel to be driven.
// Wire IN1/IN2 to two GPIO pins; which one goes high determines which
// way the motor spins.
#define PIN_ML_IN1      25      // Left motor, direction input 1 — to motor driver IN1
#define PIN_ML_IN2      26      // Left motor, direction input 2 — to motor driver IN2
#define CH_ML1          0       // LEDC channel driving PIN_ML_IN1 (0..15 on ESP32)
#define CH_ML2          1       // LEDC channel driving PIN_ML_IN2

// ---- Right motor driver (LEDC, dual-PWM) ----------------------------
#define PIN_MR_IN1      27      // Right motor, direction input 1 — to motor driver IN1
#define PIN_MR_IN2      14      // Right motor, direction input 2 — to motor driver IN2
#define CH_MR1          2       // LEDC channel driving PIN_MR_IN1
#define CH_MR2          3       // LEDC channel driving PIN_MR_IN2

// PWM settings applied to both motors.
#define PWM_FREQ_HZ     20000   // 20 kHz: above hearing range, less motor whine
#define PWM_BITS        10      // 10-bit resolution → duty 0..1023


// ---- Wheel encoders (PCNT, quadrature) ------------------------------
// Each encoder has two output lines (A and B) 90° out of phase. The
// ESP32's PCNT peripheral reads them in hardware, so no interrupts.
// The PCNT unit for each encoder is chosen in the Encoder constructor,
// not here — this file only knows the GPIO numbers.
#define PIN_EL_A        34      // Left encoder channel A — to encoder output A
#define PIN_EL_B        35      // Left encoder channel B — to encoder output B
#define PIN_ER_A        32      // Right encoder channel A — to encoder output A
#define PIN_ER_B        33      // Right encoder channel B — to encoder output B

// NOTE: on many ESP32 dev boards, GPIO 34–39 are INPUT ONLY and have no
// internal pull-ups. They work well for encoders if the encoder board
// provides its own pull-ups. Verify with the hardware team.


// ---- Encoder calibration --------------------------------------------
// ticksPerMeter turns raw encoder counts into meters of wheel travel.
// Formula: TICKS_PER_METER = (encoder_CPR × gear_ratio) / wheel_circumference_m
// Example: 11 PPR × 4 (quadrature) × 30 (gear) = 1320 CPR;
//          wheel Ø 65 mm → circumference 0.2042 m → 6464 ticks/m.
// Track width is the distance between the two wheel contact points;
// used by odometry and by differential-drive mixing.
#define TICKS_PER_METER 1234.5f // PLACEHOLDER — from encoder CPR, gear ratio, wheel Ø
#define TRACK_WIDTH_M   0.150f  // PLACEHOLDER — metres between wheel centres


// ---- Arm servos (LEDC, 50 Hz PWM) -----------------------------------
// Standard hobby servos want a 50 Hz signal with a 1–2 ms pulse.
// Wire each signal lead to its listed GPIO; power servos externally and
// connect the supply ground to ESP32 GND.
#define PIN_SHOULDER    18      // Shoulder/lift servo signal GPIO
#define CH_SHOULDER     4       // LEDC channel for the shoulder servo
#define PIN_REACH       23      // Reach servo signal GPIO
#define CH_REACH        6       // LEDC channel for reach servo
#define PIN_GRIPPER     19      // Gripper servo signal GPIO
#define CH_GRIPPER      5       // LEDC channel for the gripper servo


// ---- IMU (I2C) ------------------------------------------------------
// MPU6050 on the shared I2C bus. Address 0x68 by default (AD0 low).
// Any two GPIO pins can be the I2C bus as long as they aren't already
// used, but 21/22 are the ESP32 Arduino defaults.
#define PIN_I2C_SDA     21      // I2C data — to MPU6050 SDA
#define PIN_I2C_SCL     22      // I2C clock — to MPU6050 SCL



// ---- Control loop timing --------------------------------------------
// Fixed rates for the FreeRTOS tasks. Control and pose run at the same
// rate so they stay in lockstep; autonomy is slower because it only
// needs to react to pose, not produce it.
#define CONTROL_RATE_HZ 100     // Wheel control loop rate (Hz)
#define CONTROL_DT      (1.0f / CONTROL_RATE_HZ)  // 10 ms per tick (seconds)
#define POSE_RATE_HZ    100     // Pose estimator update rate (Hz)
#define AUTONOMY_RATE_HZ 20     // Mission state machine rate (Hz)
#define LOG_RATE_HZ     10      // Log flush rate (Hz)
#define CONTROL_PERIOD_MS (1000 / CONTROL_RATE_HZ)
#define POSE_PERIOD_MS    (1000 / POSE_RATE_HZ)
#define AUTONOMY_PERIOD_MS (1000 / AUTONOMY_RATE_HZ)


// ---- Safety limits --------------------------------------------------
// Clamps applied to operator and autonomy commands before they reach
// the wheels. Everything above is a ceiling, not a target.
#define MAX_V_MPS       0.3f    // Max forward/back speed (m/s) — PLACEHOLDER
#define MAX_W_RADPS     1.0f    // Max turn rate (rad/s) — PLACEHOLDER
#define ACCEL_V_MPS2    0.5f    // Max linear acceleration (m/s²) — ramp limit
#define ACCEL_W_RADPS2  2.0f    // Max angular acceleration (rad/s²) — ramp limit
#define ACCEL_V         ACCEL_V_MPS2
#define ACCEL_W         ACCEL_W_RADPS2
#define CMD_TIMEOUT_S   0.5f    // Stop if no drive command for this long (s)


// ---- PID gains ------------------------------------------------------
// Placeholders until bench tuning. Feed-forward does most of the work;
// the PI terms correct the residual error. KD is usually 0 on a
// first-order wheel drive.
#define KP              0.5f    // Proportional gain
#define KI              2.0f    // Integral gain
#define KD              0.0f    // Derivative gain
#define KFF             0.9f    // Feed-forward gain (power per unit setpoint)
#define SPEED_FILTER_ALPHA 0.3f // Speed low-pass: 1 = none, smaller = smoother
#define PID_KP          KP
#define PID_KI          KI
#define PID_KD          KD
#define PID_KFF         KFF
#define SPEED_FILTER    SPEED_FILTER_ALPHA
#define ENC_L_INVERTED  false
#define ENC_R_INVERTED  false


// ---- Motor behaviour ------------------------------------------------
#define MOTOR_DEADBAND  0.05f   // |power| below this stalls; compensated
#define MOTOR_SLEW      2.0f    // Max |duty/dt| in power units per second


// ---- Arm timing -----------------------------------------------------
#define ARM_TICK_MS     20      // Servo update period (ms) — 50 Hz


// ---- Logging --------------------------------------------------------
#define LOG_QUEUE_LEN   32      // Ring buffer slots; drop when full
#define LOG_PERIOD_MS   100     // Log task flush period (ms) — 10 Hz
