// PublishScheduler.h -- decides WHAT is due to be published, never publishes.
//
// Pure logic: no Arduino, no MQTT, no FreeRTOS. Runs in env:native.
// The comms task calls poll() every ~10 ms and publishes whatever is due.
//
//   telemetry  every telemetryMs (default 100 ms = 10 Hz)
//   heartbeat  every heartbeatMs (default 200 ms = 5 Hz)
//   mode       when the ModeManager epoch changed, and on every (re)connect
//              (robot/state/mode is retained, but a fresh broker session or a
//              restarted broker must be told again)
//
// Nothing is due while the link to the broker is down, so the TX queue never
// fills with stale telemetry. After a reconnect everything is due at once.
#pragma once

#include <stdint.h>
#include "robot_interfaces.h"

namespace robot {

struct SchedulerConfig {
    uint32_t telemetryMs = 100;
    uint32_t heartbeatMs = 200;
};

struct PublishDue {
    bool telemetry = false;
    bool heartbeat = false;
    bool mode      = false;
};

class PublishScheduler {
public:
    explicit PublishScheduler(const SchedulerConfig& cfg = SchedulerConfig())
        : cfg_(cfg) {}

    // modeEpoch    : ModeManager::epoch()
    // connectCount : MQTTManager::connectCount() (grows on every connect)
    // connected    : MQTTManager::isConnected()
    PublishDue poll(TimeMs now, uint32_t modeEpoch, uint32_t connectCount,
                    bool connected) {
        PublishDue d;
        if (!connected) return d;

        const bool fresh = (connectCount != lastConnectCount_) || !started_;
        if (fresh) {
            lastConnectCount_ = connectCount;
            started_ = true;
            lastTel_ = lastHb_ = now;
            lastEpoch_ = modeEpoch;
            d.telemetry = d.heartbeat = d.mode = true;
            return d;
        }

        if (modeEpoch != lastEpoch_) {
            lastEpoch_ = modeEpoch;
            d.mode = true;
        }
        d.telemetry = due(now, lastTel_, cfg_.telemetryMs);
        d.heartbeat = due(now, lastHb_, cfg_.heartbeatMs);
        return d;
    }

private:
    // Wrap-safe. Keeps the period without drift, but never bursts to "catch up"
    // after a long stall: if we are more than one period behind, resync to now.
    static bool due(TimeMs now, TimeMs& last, uint32_t period) {
        if ((uint32_t)(now - last) < period) return false;
        last += period;
        if ((uint32_t)(now - last) >= period) last = now;
        return true;
    }

    SchedulerConfig cfg_;
    bool     started_ = false;
    uint32_t lastConnectCount_ = 0;
    uint32_t lastEpoch_ = 0;
    TimeMs   lastTel_ = 0, lastHb_ = 0;
};

}  // namespace robot
