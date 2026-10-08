// ModeManager.cpp -- see ModeManager.h
#include "ModeManager.h"

namespace robot
{

    ModeManager::ModeManager()
        : state_(pack(Mode::ESTOP, EstopReason::BOOT)), epoch_(0) {}

    Mode ModeManager::current() const
    {
        return (Mode)(state_.load(std::memory_order_acquire) & 0xFFu);
    }

    EstopReason ModeManager::lastEstopReason() const
    {
        return (EstopReason)((state_.load(std::memory_order_acquire) >> 8) & 0xFFu);
    }

    uint32_t ModeManager::epoch() const
    {
        return epoch_.load(std::memory_order_acquire);
    }

    TransitionResult ModeManager::evaluate(Mode from, Mode to,
                                           const ModeGuards &g)
    {
        if (from == to)
            return TransitionResult::NO_CHANGE;

        // ESTOP is always reachable (callers normally handle it before this).
        if (to == Mode::ESTOP)
            return TransitionResult::OK;

        if (from == Mode::ESTOP)
        {
            if (to != Mode::MANUAL)
                return TransitionResult::NOT_ALLOWED;
            if (!g.linkOk)
                return TransitionResult::LINK_DOWN;
            if (!g.robotStopped)
                return TransitionResult::ROBOT_MOVING;
            return TransitionResult::OK;
        }

        // from is MANUAL, SEMI or AUTO; to is MANUAL, SEMI or AUTO.
        switch (to)
        {
        case Mode::MANUAL:
            return TransitionResult::OK;
        case Mode::SEMI:
            return g.poseValid ? TransitionResult::OK
                               : TransitionResult::POSE_INVALID;
        case Mode::AUTO:
            if (!g.poseValid)
                return TransitionResult::POSE_INVALID;
            if (!g.armIdle)
                return TransitionResult::ARM_BUSY;
            return TransitionResult::OK;
        default:
            return TransitionResult::NOT_ALLOWED;
        }
    }

    TransitionResult ModeManager::requestMode(Mode target, const ModeGuards &g,
                                              EstopReason reason)
    {
        for (;;)
        {
            uint32_t cur = state_.load(std::memory_order_acquire);
            Mode from = (Mode)(cur & 0xFFu);
            EstopReason curReason = (EstopReason)((cur >> 8) & 0xFFu);

            if (target == Mode::ESTOP)
            {
                if (from == Mode::ESTOP)
                    return TransitionResult::NO_CHANGE;
                if (state_.compare_exchange_weak(cur, pack(Mode::ESTOP, reason),
                                                 std::memory_order_acq_rel))
                {
                    epoch_.fetch_add(1, std::memory_order_acq_rel);
                    return TransitionResult::OK;
                }
                continue; // state changed under us: re-read and retry
            }

            TransitionResult r = evaluate(from, target, g);
            if (r != TransitionResult::OK)
                return r;

            // Commit only if nothing changed since we read `cur`. If ESTOP was
            // triggered meanwhile this fails, and the retry sees from == ESTOP.
            if (state_.compare_exchange_weak(cur, pack(target, curReason),
                                             std::memory_order_acq_rel))
            {
                epoch_.fetch_add(1, std::memory_order_acq_rel);
                return TransitionResult::OK;
            }
        }
    }

    const char *ModeManager::resultName(TransitionResult r)
    {
        switch (r)
        {
        case TransitionResult::OK:
            return "OK";
        case TransitionResult::NO_CHANGE:
            return "NO_CHANGE";
        case TransitionResult::NOT_ALLOWED:
            return "NOT_ALLOWED";
        case TransitionResult::POSE_INVALID:
            return "POSE_INVALID";
        case TransitionResult::ARM_BUSY:
            return "ARM_BUSY";
        case TransitionResult::LINK_DOWN:
            return "LINK_DOWN";
        case TransitionResult::ROBOT_MOVING:
            return "ROBOT_MOVING";
        }
        return "UNKNOWN";
    }

    const char *ModeManager::reasonName(EstopReason r)
    {
        switch (r)
        {
        case EstopReason::NONE:
            return "NONE";
        case EstopReason::BOOT:
            return "BOOT";
        case EstopReason::CONSOLE:
            return "CONSOLE";
        case EstopReason::BUTTON:
            return "BUTTON";
        case EstopReason::FAULT:
            return "FAULT";
        case EstopReason::WATCHDOG:
            return "WATCHDOG";
        }
        return "UNKNOWN";
    }

} // namespace robot