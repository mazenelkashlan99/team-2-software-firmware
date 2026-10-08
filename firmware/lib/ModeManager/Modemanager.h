// ModeManager.h -- the ONLY place where the robot Mode changes.
//
// Pure logic: no FreeRTOS, no Arduino.
// Thread-safety: lock-free (std::atomic compare-and-swap). Any task may call
// requestMode() or read current(). ESTOP always wins a race: if ESTOP lands
// between a transition's check and its commit, the commit fails and the
// request is re-evaluated against ESTOP.
//
// Transition table (guards listed in ModeGuards):
//   MANUAL <-> SEMI <-> AUTO      any pair, subject to entry guards
//   any mode -> ESTOP             always allowed, no guards
//   ESTOP -> MANUAL               only if link is up and robot is stopped
//   ESTOP -> SEMI / AUTO          never (must pass through MANUAL)
// Boot state: ESTOP (reason BOOT).
#pragma once

#include <stdint.h>
#include <atomic>

#include "robot_interfaces.h"

namespace robot
{

    enum class EstopReason : uint8_t
    {
        NONE = 0,
        BOOT,    // power-on state
        CONSOLE, // ESTOP button / robot/cmd/estop
        BUTTON,  // physical button (if Hardware adds one)
        FAULT,   // firmware-detected fault
        WATCHDOG // task watchdog / critical task stalled
    };

    // Snapshot of the facts the guards need. The caller fills it at call time
    // (autonomy_task reads Pose, ArmStatus and link state, then calls).
    struct ModeGuards
    {
        bool poseValid = false;    // Pose.valid (IMU calibrated, no sensor lost)
        bool armIdle = false;      // ArmStatus == IDLE
        bool linkOk = false;       // console heartbeat alive
        bool robotStopped = false; // |v| and |w| below the stopped threshold
    };

    enum class TransitionResult : uint8_t
    {
        OK = 0,
        NO_CHANGE,    // already in that mode
        NOT_ALLOWED,  // not a valid transition (e.g. ESTOP -> AUTO)
        POSE_INVALID, // guard: SEMI/AUTO need a valid pose
        ARM_BUSY,     // guard: AUTO needs the arm idle
        LINK_DOWN,    // guard: leaving ESTOP needs the console link
        ROBOT_MOVING  // guard: leaving ESTOP needs the robot stopped
    };

    class ModeManager
    {
    public:
        ModeManager();

        // Lock-free reads, safe from any task.
        Mode current() const;
        EstopReason lastEstopReason() const;
        // Increments on every successful mode change. Tasks compare it with
        // their last seen value to react (cancel GoTo, zero the Command, ...).
        uint32_t epoch() const;

        // The single validated transition function.
        // target == ESTOP : always succeeds (first reason is kept if already ESTOP)
        // from ESTOP      : only to MANUAL, with the ESTOP-release guards
        // otherwise       : see the table above
        TransitionResult requestMode(Mode target,
                                     const ModeGuards &guards = ModeGuards(),
                                     EstopReason reason = EstopReason::CONSOLE);

        // Convenience for the safety path.
        TransitionResult triggerEstop(EstopReason reason)
        {
            return requestMode(Mode::ESTOP, ModeGuards(), reason);
        }

        // Pure decision function (no state), public so it can be unit-tested.
        static TransitionResult evaluate(Mode from, Mode to,
                                         const ModeGuards &guards);

        static const char *resultName(TransitionResult r);
        static const char *reasonName(EstopReason r);

    private:
        // 32-bit atomics only: native on the ESP32 (no libatomic needed).
        // state_ layout: bits 0-7 = Mode, bits 8-15 = EstopReason.
        static uint32_t pack(Mode m, EstopReason r)
        {
            return (uint32_t)m | ((uint32_t)r << 8);
        }
        std::atomic<uint32_t> state_;
        std::atomic<uint32_t> epoch_;
    };

} // namespace robot