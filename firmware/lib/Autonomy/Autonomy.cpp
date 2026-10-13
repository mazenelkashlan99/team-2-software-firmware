#include "Autonomy.h"

void Autonomy::begin() {
    state_   = State::Idle;
    nextBox_ = 0;
}

Autonomy::Step Autonomy::step(const robot::Pose& /*pose*/,
                              robot::ArmStatus arm,
                              const robot::QrResult& /*qr*/,
                              const robot::MissionConfig& mission) {
    Step s;

    switch (state_) {
        case State::Idle:
            if (mission.boxCount == 0) { s.action = Step::Action::Idle; break; }
            s.action  = Step::Action::DriveTo;
            s.targetX = mission.boxes[nextBox_].x;
            s.targetY = mission.boxes[nextBox_].y;
            state_ = State::GoToBox;
            break;

        case State::GoToBox:
            // TODO: check pose proximity to the box before picking.
            s.action = Step::Action::ArmPick;
            state_   = State::Pick;
            break;

        case State::Pick:
            if (arm == robot::ArmStatus::DONE) state_ = State::GoToZone;
            s.action = Step::Action::Idle;
            break;

        case State::GoToZone:
            // TODO: pick the zone matching the box's colour.
            s.action = Step::Action::ArmDrop;
            state_   = State::Drop;
            break;

        case State::Drop:
            if (arm == robot::ArmStatus::DONE) {
                nextBox_++;
                state_ = (nextBox_ >= mission.boxCount) ? State::Done
                                                        : State::Idle;
            }
            s.action = Step::Action::Idle;
            break;

        case State::Done:
            s.action = Step::Action::Done;
            break;
    }
    return s;
}