#pragma once
#include "robot_interfaces.h"

// Mission state machine. Pure policy: no side effects, no hardware.
// Returns a small action for autonomy_task to execute.
//
// This is a skeleton. The real logic — pose proximity, QR destination,
// zone selection by colour — is filled in once the field is measured.
class Autonomy {
public:
    void begin();

    struct Step {
        enum class Action {
            Idle,
            DriveTo,
            Stop,
            ArmPick,
            ArmDrop,
            ArmStow,
            Done
        };
        Action action  = Action::Idle;
        float  targetX = 0.0f;
        float  targetY = 0.0f;
    };

    Step step(const robot::Pose& pose,
              robot::ArmStatus    arm,
              const robot::QrResult& qr,
              const robot::MissionConfig& mission);

private:
    enum class State { Idle, GoToBox, Pick, GoToZone, Drop, Done };

    State   state_   = State::Idle;
    uint8_t nextBox_ = 0;
};