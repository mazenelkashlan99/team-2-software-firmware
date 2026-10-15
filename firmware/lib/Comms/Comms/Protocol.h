#pragma once
#include <stdint.h>
#include <stddef.h>
#include "robot_interfaces.h"
#include "Topics.h"

namespace robot {
namespace proto {

// ---- Version / limits -------------------------------------------------
constexpr int    PROTOCOL_VERSION = 1;
constexpr size_t SID_MAX          = 16;
constexpr size_t MAX_PAYLOAD      = 1024;
constexpr size_t QR_RAW_MAX       = 32;
constexpr float  FIELD_MIN_M      = -0.5f;   // pose range on the field
constexpr float  FIELD_MAX_M      =  3.5f;

// ---- Envelope ---------------------------------------------------------
struct Envelope {
    int      ver = PROTOCOL_VERSION;
    char     sid[SID_MAX] = {0};
    uint32_t seq = 0;
};

// ---- Message types ----------------------------------------------------
enum class MsgType : uint8_t {
    UNKNOWN,
    DRIVE,
    HEARTBEAT,
    MODE,
    ESTOP,
    ESTOP_RESET,
    START,
    MISSION,
    GOTO,
    ARM,
    POSE_SET,
    PING,
    CV_QR,
    CV_COLOR,
};

enum class Sender : uint8_t {
    CONSOLE = 0,
    CV      = 1,
};

enum class CvColor : uint8_t {
    NONE = 0,
    BLUE = 1,
    RED  = 2,
};

enum class DecodeError : uint8_t {
    OK = 0,
    BAD_JSON,
    BAD_ENVELOPE,
    MISSING_FIELD,
    BAD_VALUE,
    UNKNOWN_TOPIC,
};

// ---- Decoded message --------------------------------------------------
struct Message {
    MsgType  type = MsgType::UNKNOWN;
    Envelope env  {};

    // DRIVE
    float thr  = 0.0f;
    float turn = 0.0f;

    // MODE
    robot::Mode mode = robot::Mode::ESTOP;

    // MISSION
    robot::MissionConfig mission {};

    // GOTO
    float gotoX = 0.0f;
    float gotoY = 0.0f;

    // ARM
    robot::ArmCmd arm = robot::ArmCmd::STOW;

    // POSE_SET
    float poseX = 0.0f;
    float poseY = 0.0f;
    float poseTheta = 0.0f;

    // PING / PONG
    int32_t pingId = 0;

    // CV
    robot::QrResult qr {};
    bool     colorValid = false;
    CvColor  color      = CvColor::NONE;
};

// ---- Telemetry (robot -> console) -------------------------------------
struct Telemetry {
    robot::Pose      pose {};
    robot::Mode      mode = robot::Mode::ESTOP;
    robot::ArmStatus arm  = robot::ArmStatus::IDLE;
    const char*      autoState = nullptr;
    float            battV = 0.0f;
    bool             linkOk = false;
};

// ---- Lookups ----------------------------------------------------------
MsgType     typeFromTopic(const char* topic);
Sender      senderOf(MsgType type);
const char* errorName(DecodeError e);
const char* modeName(robot::Mode m);
const char* armStatusName(robot::ArmStatus s);

// ---- QR string parser: "X=1.56&Y=0.49" --------------------------------
bool parseQrString(const char* s, float& x, float& y);

// ---- Decode (topic + payload -> Message) ------------------------------
DecodeError decode(const char* topicName, const char* payload, size_t len,
                   Message& out);

// ---- Encoders ---------------------------------------------------------
size_t encodeTelemetry(char* buf, size_t cap, const Envelope& env,
                       TimeMs t, const Telemetry& tel);
size_t encodeModeState(char* buf, size_t cap, const Envelope& env,
                       TimeMs t, robot::Mode m);
size_t encodeHeartbeat(char* buf, size_t cap, const Envelope& env, TimeMs t);
size_t encodePong(char* buf, size_t cap, const Envelope& env,
                  TimeMs t, int32_t pingId);
size_t encodeEvent(char* buf, size_t cap, const Envelope& env,
                   TimeMs t, const char* ev, const char* detail);
size_t encodeReject(char* buf, size_t cap, const Envelope& env,
                    TimeMs t, const char* rejectedTopic, DecodeError reason);
size_t encodeLog(char* buf, size_t cap, const Envelope& env,
                 TimeMs t, const char* level, const char* msg);

} // namespace proto
} // namespace robot