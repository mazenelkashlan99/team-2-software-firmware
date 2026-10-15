#include "control_task.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <driver/pcnt.h>

#include "config.h"
#include "comms_task.h"        // g_driveQ, DriveCmd, g_modeManager
#include "arm_task.h"          // armRequestAbort()
#include "Motor-Controller.h"  // class MotorController (adjust to your file name)
#include "Encoder.h"
#include "PID.h"
#include "WheelController.h"
#include "DriveBase.h"

namespace {

struct AutoCmd { float v; float w; };

// Constructors only store arguments, so global construction is safe.
MotorController motorL(PIN_ML_IN1, PIN_ML_IN2, CH_ML1, CH_ML2);
MotorController motorR(PIN_MR_IN1, PIN_MR_IN2, CH_MR1, CH_MR2);
Encoder encL(PCNT_UNIT_0, PIN_EL_A, PIN_EL_B);
Encoder encR(PCNT_UNIT_1, PIN_ER_A, PIN_ER_B);
PID pidL, pidR;
WheelController wheelL(motorL, encL, pidL, TICKS_PER_METER);
WheelController wheelR(motorR, encR, pidR, TICKS_PER_METER);
DriveBase base(wheelL, wheelR, TRACK_WIDTH_M);

QueueHandle_t s_autoQ = nullptr;
portMUX_TYPE  s_mux   = portMUX_INITIALIZER_UNLOCKED;
WheelState    s_shared;

void setupWheel(MotorController& m, Encoder& e, PID& p, WheelController& w, bool inverted) {
    m.begin(PWM_FREQ_HZ, PWM_BITS);
    m.setDeadband(MOTOR_DEADBAND);
    m.setSlewRate(MOTOR_SLEW);
    e.begin();
    p.setGains(PID_KP, PID_KI, PID_KD, PID_KFF);
    p.setOutputLimits(-1.0f, 1.0f);
    p.setIntegralLimits(-1.0f, 1.0f);
    w.setInverted(inverted);
    w.setSpeedFilter(SPEED_FILTER);
}

}  // namespace

void controlInit() {
    setupWheel(motorL, encL, pidL, wheelL, ENC_L_INVERTED);
    setupWheel(motorR, encR, pidR, wheelR, ENC_R_INVERTED);
    base.setLimits(MAX_V_MPS, MAX_W_RADPS);
    base.setRamps(ACCEL_V, ACCEL_W);
    base.setCommandTimeout(CMD_TIMEOUT_S);
    s_autoQ = xQueueCreate(1, sizeof(AutoCmd));
}

void controlPostAutoCommand(float v, float w) {
    AutoCmd c{v, w};
    xQueueOverwrite(s_autoQ, &c);     // only the newest matters
}

float wheelDistanceDeltaLeft()  { return wheelL.getDistanceDelta(); }
float wheelDistanceDeltaRight() { return wheelR.getDistanceDelta(); }

WheelState controlGetWheelState() {
    WheelState s;
    portENTER_CRITICAL(&s_mux);
    s = s_shared;
    portEXIT_CRITICAL(&s_mux);
    return s;
}

void controlTask(void*) {
    esp_task_wdt_add(nullptr);                   // a hung control loop resets the board
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastUs = micros();
    bool estopApplied = false;
    uint32_t tick = 0;

    for (;;) {
        // Measured dt (the loop jitters slightly), clamped so one bad tick can't break the PID.
        const uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;
        if (dt < 0.005f) dt = 0.005f;
        if (dt > 0.050f) dt = 0.050f;

        const robot::Mode mode = g_modeManager.current();
        DriveCmd d;
        AutoCmd  a;

        if (mode == robot::Mode::ESTOP) {
            if (!estopApplied) {                 // act once, on entry
                base.emergencyStop();
                armRequestAbort();
                estopApplied = true;
            }
            xQueueReceive(g_driveQ, &d, 0);      // discard anything stale
            xQueueReceive(s_autoQ, &a, 0);
        } else {
            if (estopApplied) {                  // left ESTOP: stay still until a NEW command
                base.clearEmergencyStop();
                estopApplied = false;
            }
            if (mode == robot::Mode::MANUAL) {
                if (xQueueReceive(g_driveQ, &d, 0) == pdTRUE)
                    base.setCommand(d.thr * MAX_V_MPS, d.turn * MAX_W_RADPS);
                xQueueReceive(s_autoQ, &a, 0);   // ignore autonomous commands in MANUAL
            } else {                             // SEMI or AUTO
                if (xQueueReceive(s_autoQ, &a, 0) == pdTRUE)
                    base.setCommand(a.v, a.w);
                xQueueReceive(g_driveQ, &d, 0);  // no stale joystick command after switching back
            }
        }

        base.update(dt);                         // ramps -> both wheel loops -> motors

        WheelState s;
        s.totalL = wheelL.getTotalDistance();
        s.totalR = wheelR.getTotalDistance();
        s.speedL = wheelL.getSpeed();
        s.speedR = wheelR.getSpeed();
        s.tick   = ++tick;
        portENTER_CRITICAL(&s_mux);
        s_shared = s;
        portEXIT_CRITICAL(&s_mux);

        esp_task_wdt_reset();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
