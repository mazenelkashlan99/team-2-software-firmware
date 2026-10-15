// comms_core.h -- what the robot does with the messages the console sends.
//
// Pure logic: no Wi-Fi, no MQTT client, no FreeRTOS. Runs in env:native.
// The ESP32 glue (comms_task.cpp) connects it to MQTTManager.
//
//   RX  onMessage()  decode -> duplicate filter -> mode rules -> action
//   OUT getCommand() the {v, w} for the drive (Person A reads it at >= 20 Hz)
//   TX  tick()       telemetry 10 Hz, heartbeat 5 Hz, mode on change
//
// Threads:  onMessage()  : MQTT task only
//           tick()       : comms TX task only
//           getCommand() / linkOk() : any task (lock-free)
// Manual driving only: modes MANUAL and ESTOP.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <atomic>

#include "robot_interfaces.h"
#include "ModeManager.h"
#include "Protocol.h"
#include "PublishScheduler.h"

namespace robot {

// Where outgoing messages go (MQTTManager on the robot, a fake in the tests).
class IPublisher {
public:
    virtual ~IPublisher() {}
    virtual bool publish(const char* topic, const char* payload, size_t len,
                         bool retain, bool critical) = 0;
};

// What the rest of the firmware must provide. Any pointer may be null.
struct CommsPorts {
    void (*emergencyStop)()       = nullptr;  // A: stop the motors NOW.
                                              //   Must not block, any task.
    void (*armAction)(ArmCmd)     = nullptr;  // A: start an arm macro
};

// Data for telemetry. Any pointer may be null (safe defaults).
struct CommsSources {
    Pose      (*pose)()  = nullptr;   // B: current pose
    ArmStatus (*arm)()   = nullptr;   // A: arm status
    float     (*battV)() = nullptr;   // volts, NAN if unknown
};

struct CommsConfig {
    // PLACEHOLDERS until A gives the real limits (config.h).
    float    vMax           = 0.5f;   // m/s at thr = 1
    float    wMax           = 2.0f;   // rad/s at turn = 1
    uint32_t driveTimeoutMs = 300;    // no drive message -> command zero
    uint32_t linkTimeoutMs  = 300;    // no console message -> link down
    float    stoppedV       = 0.02f;  // m/s  : "stopped" for ESTOP release
    float    stoppedW       = 0.10f;  // rad/s
    SchedulerConfig sched;
};

class CommsCore {
public:
    // sid: this robot's id for the session (<= 15 chars).
    CommsCore(ModeManager& modes, const CommsPorts& ports,
              const CommsSources& src, const CommsConfig& cfg,
              const char* sid, IPublisher* pub);

    // One received MQTT message. Short and non-blocking.
    void onMessage(const char* topic, const char* payload, size_t len,
                   TimeMs now);

    // Command for the drive. Zero in ESTOP, when no drive message arrived in
    // the last driveTimeoutMs, or before the first one.
    Command getCommand(TimeMs now) const;

    // true if any console message arrived in the last linkTimeoutMs.
    bool linkOk(TimeMs now) const;

    // Call every ~10 ms from the TX task.
    void tick(TimeMs now, uint32_t connectCount, bool connected);

    // Robot envelope: ver, sid, +1 seq. Thread-safe.
    proto::Envelope nextEnvelope();

private:
    void clearDrive();
    bool robotStopped() const;
    void publishJson(const char* topic, const char* buf, size_t n,
                     bool retain, bool critical);
    void publishEvent(const char* ev, const char* detail, TimeMs now);
    void publishReject(const char* topic, const char* reason, TimeMs now);

    ModeManager&  modes_;
    CommsPorts    ports_;
    CommsSources  src_;
    CommsConfig   cfg_;
    char          sid_[proto::SID_MAX];
    IPublisher*   pub_;

    proto::SeqFilter console_;       // MQTT task only
    PublishScheduler sched_;         // TX task only

    std::atomic<uint32_t> txSeq_;
    // Drive slot: thr and turn as int16 packed in one 32-bit atomic, so the
    // two values are always read together.
    std::atomic<uint32_t> drivePacked_;
    std::atomic<uint32_t> driveMs_;
    std::atomic<bool>     haveDrive_;
    std::atomic<uint32_t> linkMs_;
    std::atomic<bool>     haveLink_;
};

}  // namespace robot
