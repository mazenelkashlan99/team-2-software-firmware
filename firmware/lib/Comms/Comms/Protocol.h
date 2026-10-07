// Protocol.h -- decode/encode of MQTT payloads (JSON, ArduinoJson v7)
//
// Pure logic: no Wi-Fi, no MQTT client, no FreeRTOS. Runs in env:native.
// ArduinoJson is used only inside Protocol.cpp, so no other module needs it.
//
// Flow:  MQTT callback -> decode(topic, payload) -> Message
//        -> SeqFilter::accept(msg.env) -> post event to the autonomy task.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

#include "robot_interfaces.h"
#include "Topics.h"

namespace robot
{
    namespace proto
    {

        static const uint8_t PROTOCOL_VERSION = 1;
        static const size_t SID_MAX = 16;       // sender id, including NUL
        static const size_t MAX_PAYLOAD = 1024; // bytes; larger = rejected

        // Field limits [PROPOSAL: move to config.h when A and C share it]
        static const float FIELD_MIN_M = 0.0f;
        static const float FIELD_MAX_M = 3.0f;

        enum class MsgType : uint8_t
        {
            UNKNOWN = 0,
            DRIVE,       // robot/cmd/drive
            HEARTBEAT,   // robot/hb/console
            MODE,        // robot/cmd/mode
            ESTOP,       // robot/cmd/estop
            ESTOP_RESET, // robot/cmd/estop_reset
            START,       // robot/cmd/start
            MISSION,     // robot/cmd/mission
            GOTO,        // robot/cmd/goto
            ARM,         // robot/cmd/arm
            POSE_SET,    // robot/cmd/pose_set
            PING,        // robot/cmd/ping
            CV_QR,       // robot/cv/qr
            CV_COLOR     // robot/cv/color
        };

        enum class DecodeError : uint8_t
        {
            OK = 0,
            BAD_JSON,      // not JSON, empty, or larger than MAX_PAYLOAD
            BAD_ENVELOPE,  // ver/sid/seq missing or invalid
            MISSING_FIELD, // a required field is absent
            BAD_VALUE,     // wrong type, out of range, unknown string
            UNKNOWN_TOPIC
        };

        // Independent sequence spaces: the console and the CV script have their own
        // sid/seq, so each gets its own SeqFilter.
        enum class Sender : uint8_t
        {
            CONSOLE = 0,
            CV = 1
        };

        enum class CvColor : uint8_t
        {
            NONE = 0,
            BLUE,
            RED
        };

        struct Envelope
        {
            uint8_t ver = 0;
            char sid[SID_MAX] = {0};
            uint32_t seq = 0;
        };

        // One decoded message. Only the fields for `type` are meaningful.
        struct Message
        {
            MsgType type = MsgType::UNKNOWN;
            Envelope env;

            float thr = 0.0f, turn = 0.0f;    // DRIVE   (-1..1)
            Mode mode = Mode::MANUAL;         // MODE    (never ESTOP)
            float gotoX = 0.0f, gotoY = 0.0f; // GOTO    (m)
            ArmCmd arm = ArmCmd::STOW;        // ARM
            float poseX = 0.0f, poseY = 0.0f,
                  poseTheta = 0.0f;  // POSE_SET (m, m, rad)
            int32_t pingId = 0;      // PING
            MissionConfig mission;   // MISSION
            QrResult qr;             // CV_QR
            bool colorValid = false; // CV_COLOR
            CvColor color = CvColor::NONE;
        };

        // ---- Decoding ---------------------------------------------------------------
        MsgType typeFromTopic(const char *topic); // UNKNOWN if not recognised
        Sender senderOf(MsgType type);
        DecodeError decode(const char *topic, const char *payload, size_t len,
                           Message &out);
        const char *errorName(DecodeError e);

        // "X=1.56&Y=0.49" -> x, y. Strict format; false on anything else.
        bool parseQrString(const char *s, float &x, float &y);

        // ---- Duplicate / stale filter ----------------------------------------------
        // accept() returns true if the message is new. A different sid means the
        // sender restarted: tracking resets and the message is accepted.
        class SeqFilter
        {
        public:
            bool accept(const Envelope &e)
            {
                if (!has_ || strncmp(sid_, e.sid, SID_MAX) != 0)
                {
                    strncpy(sid_, e.sid, SID_MAX - 1);
                    sid_[SID_MAX - 1] = '\0';
                    last_ = e.seq;
                    has_ = true;
                    return true;
                }
                if (e.seq > last_)
                {
                    last_ = e.seq;
                    return true;
                }
                return false; // duplicate or older
            }
            void reset()
            {
                has_ = false;
                last_ = 0;
                sid_[0] = '\0';
            }

        private:
            char sid_[SID_MAX] = {0};
            uint32_t last_ = 0;
            bool has_ = false;
        };

        // ---- Encoding (robot -> console) -------------------------------------------
        // Every encoder writes a NUL-terminated JSON string into buf and returns the
        // number of bytes (without NUL), or 0 if buf is too small. env = the robot's
        // own sid/seq (caller increments seq per message); t = ms since boot.
        struct Telemetry
        {
            Pose pose; // x y theta v w valid
            Mode mode = Mode::ESTOP;
            const char *autoState = "IDLE"; // autonomy state name
            ArmStatus arm = ArmStatus::IDLE;
            float battV = NAN; // NaN -> JSON null (unknown)
            bool linkOk = false;
        };

        const char *modeName(Mode m);
        const char *armStatusName(ArmStatus s);

        size_t encodeTelemetry(char *buf, size_t cap, const Envelope &env, TimeMs t,
                               const Telemetry &tel);
        size_t encodeModeState(char *buf, size_t cap, const Envelope &env, TimeMs t,
                               Mode m);
        size_t encodeHeartbeat(char *buf, size_t cap, const Envelope &env, TimeMs t);
        size_t encodePong(char *buf, size_t cap, const Envelope &env, TimeMs t,
                          int32_t pingId);
        size_t encodeEvent(char *buf, size_t cap, const Envelope &env, TimeMs t,
                           const char *ev, const char *detail);
        size_t encodeReject(char *buf, size_t cap, const Envelope &env, TimeMs t,
                            const char *rejectedTopic, DecodeError reason);
        size_t encodeLog(char *buf, size_t cap, const Envelope &env, TimeMs t,
                         const char *level, const char *msg);

    } // namespace proto
} // namespace robot