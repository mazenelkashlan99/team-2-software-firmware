// comms_core.cpp -- see comms_core.h
#include "comms_core.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

namespace robot {

using namespace proto;

static uint32_t packDrive(float thr, float turn) {
    int16_t a = (int16_t)lrintf(thr  * 32767.0f);
    int16_t b = (int16_t)lrintf(turn * 32767.0f);
    return ((uint32_t)(uint16_t)a << 16) | (uint16_t)b;
}

CommsCore::CommsCore(ModeManager& modes, const CommsPorts& ports,
                     const CommsSources& src, const CommsConfig& cfg,
                     const char* sid, IPublisher* pub)
    : modes_(modes), ports_(ports), src_(src), cfg_(cfg), pub_(pub),
      sched_(cfg.sched), txSeq_(0), drivePacked_(0), driveMs_(0),
      haveDrive_(false), linkMs_(0), haveLink_(false) {
    strncpy(sid_, sid ? sid : "robot", SID_MAX - 1);
    sid_[SID_MAX - 1] = '\0';
}

Envelope CommsCore::nextEnvelope() {
    Envelope e;
    e.ver = PROTOCOL_VERSION;
    strncpy(e.sid, sid_, SID_MAX - 1);
    e.sid[SID_MAX - 1] = '\0';
    e.seq = txSeq_.fetch_add(1);
    return e;
}

// ---- helpers ----------------------------------------------------------------

void CommsCore::clearDrive() {
    haveDrive_.store(false);
    drivePacked_.store(0);
}

bool CommsCore::robotStopped() const {
    if (!src_.pose) return true;            // in ESTOP the motors are at zero
    Pose p = src_.pose();
    if (!p.valid) return true;              // no estimate: trust the zero command
    return fabsf(p.v) <= cfg_.stoppedV && fabsf(p.w) <= cfg_.stoppedW;
}

void CommsCore::publishJson(const char* topic, const char* buf, size_t n,
                            bool retain, bool critical) {
    if (pub_ && n > 0) pub_->publish(topic, buf, n, retain, critical);
}

void CommsCore::publishEvent(const char* ev, const char* detail, TimeMs now) {
    char buf[256];
    size_t n = encodeEvent(buf, sizeof buf, nextEnvelope(), now, ev, detail);
    publishJson(topic::STATUS_EVENT, buf, n, false, false);
}

// State-based refusal: ev CMD_REJECTED, detail "<topic>:<REASON>".
void CommsCore::publishReject(const char* topicName, const char* reason,
                              TimeMs now) {
    char detail[64];
    snprintf(detail, sizeof detail, "%s:%s", topicName, reason);
    publishEvent("CMD_REJECTED", detail, now);
}

// ---- RX ---------------------------------------------------------------------

void CommsCore::onMessage(const char* topicName, const char* payload,
                          size_t len, TimeMs now) {
    Message msg;
    DecodeError err = decode(topicName, payload, len, msg);
    if (err != DecodeError::OK) {
        char buf[256];
        size_t n = encodeReject(buf, sizeof buf, nextEnvelope(), now,
                                topicName, err);
        publishJson(topic::STATUS_EVENT, buf, n, false, false);
        return;
    }
    if (!console_.accept(msg.env)) return;      // duplicate or old

    linkMs_.store(now);
    haveLink_.store(true);

    switch (msg.type) {
        case MsgType::HEARTBEAT:
            break;                              // link already refreshed

        case MsgType::DRIVE:
            if (modes_.current() == Mode::MANUAL) {
                drivePacked_.store(packDrive(msg.thr, msg.turn));
                driveMs_.store(now);
                haveDrive_.store(true);
            }                                   // ESTOP: ignored silently
            break;

        case MsgType::ESTOP:
            modes_.triggerEstop(EstopReason::CONSOLE);
            clearDrive();
            if (ports_.emergencyStop) ports_.emergencyStop();
            break;

        case MsgType::ESTOP_RESET: {
            if (modes_.current() != Mode::ESTOP) break;     // ignored
            ModeGuards g;
            g.linkOk = linkOk(now);
            g.robotStopped = robotStopped();
            TransitionResult r =
                modes_.requestMode(Mode::MANUAL, g, EstopReason::CONSOLE);
            if (r == TransitionResult::OK) clearDrive();
            else publishReject(topic::CMD_ESTOP_RESET,
                               ModeManager::resultName(r), now);
            break;
        }

        case MsgType::ARM:
            if (modes_.current() == Mode::MANUAL) {
                if (ports_.armAction) ports_.armAction(msg.arm);
            } else {
                publishReject(topic::CMD_ARM, "NOT_ALLOWED", now);
            }
            break;

        case MsgType::PING: {
            char buf[128];
            size_t n = encodePong(buf, sizeof buf, nextEnvelope(), now,
                                  msg.pingId);
            publishJson(topic::STATE_PONG, buf, n, false, false);
            break;
        }

        default:
            break;      // topics of the dropped modes: not subscribed anyway
    }
}

// ---- OUT --------------------------------------------------------------------

Command CommsCore::getCommand(TimeMs now) const {
    Command c;                                   // zero
    if (modes_.current() != Mode::MANUAL) return c;
    if (!haveDrive_.load()) return c;
    if ((TimeMs)(now - driveMs_.load()) > cfg_.driveTimeoutMs) return c;
    uint32_t p = drivePacked_.load();
    float thr  = (int16_t)(p >> 16)    / 32767.0f;
    float turn = (int16_t)(p & 0xFFFF) / 32767.0f;
    c.v = thr  * cfg_.vMax;
    c.w = turn * cfg_.wMax;
    return c;
}

bool CommsCore::linkOk(TimeMs now) const {
    return haveLink_.load() &&
           (TimeMs)(now - linkMs_.load()) <= cfg_.linkTimeoutMs;
}

// ---- TX ---------------------------------------------------------------------

void CommsCore::tick(TimeMs now, uint32_t connectCount, bool connected) {
    PublishDue d = sched_.poll(now, modes_.epoch(), connectCount, connected);
    char buf[512];

    if (d.mode) {       // retained + critical: survives a reconnect
        size_t n = encodeModeState(buf, sizeof buf, nextEnvelope(), now,
                                   modes_.current());
        publishJson(topic::STATE_MODE, buf, n, true, true);
    }
    if (d.telemetry) {
        Telemetry t;
        if (src_.pose)  t.pose = src_.pose();
        if (src_.arm)   t.arm  = src_.arm();
        if (src_.battV) t.battV = src_.battV();
        t.mode   = modes_.current();
        t.linkOk = linkOk(now);
        size_t n = encodeTelemetry(buf, sizeof buf, nextEnvelope(), now, t);
        publishJson(topic::STATE_TELEMETRY, buf, n, false, false);
    }
    if (d.heartbeat) {
        size_t n = encodeHeartbeat(buf, sizeof buf, nextEnvelope(), now);
        publishJson(topic::HB_ROBOT, buf, n, false, false);
    }
}

}  // namespace robot
