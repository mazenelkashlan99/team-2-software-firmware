// Protocol.cpp -- see Protocol.h
#include "Protocol.h"

#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

namespace robot
{
    namespace proto
    {

        namespace
        {

#define TRY(expr)                  \
    do                             \
    {                              \
        DecodeError e_ = (expr);   \
        if (e_ != DecodeError::OK) \
            return e_;             \
    } while (0)

            // --- field readers -----------------------------------------------------------
            DecodeError readFloat(JsonVariantConst v, float lo, float hi, float &out)
            {
                if (v.isNull())
                    return DecodeError::MISSING_FIELD;
                if (!v.is<float>())
                    return DecodeError::BAD_VALUE; // int or float
                float f = v.as<float>();
                if (!std::isfinite(f) || f < lo || f > hi)
                    return DecodeError::BAD_VALUE;
                out = f;
                return DecodeError::OK;
            }

            DecodeError readInt(JsonVariantConst v, long lo, long hi, long &out)
            {
                if (v.isNull())
                    return DecodeError::MISSING_FIELD;
                if (!v.is<long>())
                    return DecodeError::BAD_VALUE;
                long n = v.as<long>();
                if (n < lo || n > hi)
                    return DecodeError::BAD_VALUE;
                out = n;
                return DecodeError::OK;
            }

            DecodeError readString(JsonVariantConst v, const char *&out)
            {
                if (v.isNull())
                    return DecodeError::MISSING_FIELD;
                if (!v.is<const char *>())
                    return DecodeError::BAD_VALUE;
                out = v.as<const char *>();
                return DecodeError::OK;
            }

            DecodeError readBool(JsonVariantConst v, bool &out)
            {
                if (v.isNull())
                    return DecodeError::MISSING_FIELD;
                if (!v.is<bool>())
                    return DecodeError::BAD_VALUE;
                out = v.as<bool>();
                return DecodeError::OK;
            }

            DecodeError parseEnvelope(JsonObjectConst root, Envelope &env)
            {
                JsonVariantConst ver = root["ver"];
                if (!ver.is<int>() || ver.as<int>() != PROTOCOL_VERSION)
                    return DecodeError::BAD_ENVELOPE;

                JsonVariantConst sid = root["sid"];
                if (!sid.is<const char *>())
                    return DecodeError::BAD_ENVELOPE;
                const char *s = sid.as<const char *>();
                size_t n = strlen(s);
                if (n == 0 || n >= SID_MAX)
                    return DecodeError::BAD_ENVELOPE;

                JsonVariantConst seq = root["seq"];
                if (!seq.is<uint32_t>())
                    return DecodeError::BAD_ENVELOPE;

                env.ver = PROTOCOL_VERSION;
                memcpy(env.sid, s, n + 1);
                env.seq = seq.as<uint32_t>();
                return DecodeError::OK;
            }

            bool boxTypeFromString(const char *s, BoxType &out)
            {
                if (strcmp(s, "BLUE") == 0)
                {
                    out = BoxType::BLUE_EASY;
                    return true;
                }
                if (strcmp(s, "RED") == 0)
                {
                    out = BoxType::RED_HARD;
                    return true;
                }
                if (strcmp(s, "QR") == 0)
                {
                    out = BoxType::QR;
                    return true;
                }
                return false;
            }

            DecodeError decodeMission(JsonObjectConst root, MissionConfig &m)
            {
                JsonArrayConst boxes = root["boxes"].as<JsonArrayConst>();
                if (boxes.isNull())
                    return DecodeError::MISSING_FIELD;
                JsonArrayConst zones = root["zones"].as<JsonArrayConst>();
                if (zones.isNull())
                    return DecodeError::MISSING_FIELD;
                if (boxes.size() > MAX_BOXES || zones.size() > MAX_ZONES)
                    return DecodeError::BAD_VALUE;

                uint8_t n = 0;
                for (JsonObjectConst b : boxes)
                {
                    if (b.isNull())
                        return DecodeError::BAD_VALUE;
                    long id = 0;
                    TRY(readInt(b["id"], 0, 255, id));
                    const char *ts = nullptr;
                    TRY(readString(b["type"], ts));
                    BoxType bt;
                    if (!boxTypeFromString(ts, bt))
                        return DecodeError::BAD_VALUE;
                    float x = 0, y = 0;
                    TRY(readFloat(b["x"], FIELD_MIN_M, FIELD_MAX_M, x));
                    TRY(readFloat(b["y"], FIELD_MIN_M, FIELD_MAX_M, y));
                    m.boxes[n].id = (uint8_t)id;
                    m.boxes[n].type = bt;
                    m.boxes[n].x = x;
                    m.boxes[n].y = y;
                    n++;
                }
                m.boxCount = n;

                uint8_t z = 0;
                for (JsonObjectConst zo : zones)
                {
                    if (zo.isNull())
                        return DecodeError::BAD_VALUE;
                    const char *ts = nullptr;
                    TRY(readString(zo["type"], ts));
                    BoxType bt;
                    if (!boxTypeFromString(ts, bt) || bt == BoxType::QR)
                        return DecodeError::BAD_VALUE; // QR boxes have no zone
                    float x = 0, y = 0;
                    TRY(readFloat(zo["x"], FIELD_MIN_M, FIELD_MAX_M, x));
                    TRY(readFloat(zo["y"], FIELD_MIN_M, FIELD_MAX_M, y));
                    m.zones[z].type = bt;
                    m.zones[z].x = x;
                    m.zones[z].y = y;
                    z++;
                }
                m.zoneCount = z;
                return DecodeError::OK;
            }

            DecodeError decodeCvQr(JsonObjectConst root, const Envelope &env,
                                   QrResult &qr)
            {
                bool valid = false;
                TRY(readBool(root["valid"], valid));
                qr = QrResult();
                qr.seq = env.seq;
                if (!valid)
                    return DecodeError::OK; // "no QR visible"

                const char *raw = nullptr;
                TRY(readString(root["raw"], raw));
                if (strlen(raw) >= QR_RAW_MAX)
                    return DecodeError::BAD_VALUE;

                float px = 0, py = 0;
                if (!parseQrString(raw, px, py))
                    return DecodeError::BAD_VALUE;
                if (px < FIELD_MIN_M || px > FIELD_MAX_M ||
                    py < FIELD_MIN_M || py > FIELD_MAX_M)
                    return DecodeError::BAD_VALUE; // outside the 3 x 3 m field

                float x = 0, y = 0;
                TRY(readFloat(root["x"], FIELD_MIN_M, FIELD_MAX_M, x));
                TRY(readFloat(root["y"], FIELD_MIN_M, FIELD_MAX_M, y));
                // CV's parsed numbers must agree with the raw string (1 cm tolerance).
                if (std::fabs(x - px) > 0.01f || std::fabs(y - py) > 0.01f)
                    return DecodeError::BAD_VALUE;

                qr.valid = true;
                qr.x = px;
                qr.y = py;
                strncpy(qr.raw, raw, QR_RAW_MAX - 1);
                qr.raw[QR_RAW_MAX - 1] = '\0';
                return DecodeError::OK;
            }

            // --- encoding helpers --------------------------------------------------------
            float round3(float v) { return std::round(v * 1000.0f) / 1000.0f; }

            void stamp(JsonDocument &doc, const Envelope &env, TimeMs t)
            {
                doc["ver"] = PROTOCOL_VERSION;
                doc["sid"] = env.sid;
                doc["seq"] = env.seq;
                doc["t"] = t;
            }

            size_t finish(const JsonDocument &doc, char *buf, size_t cap)
            {
                if (buf == nullptr || cap == 0)
                    return 0;
                size_t need = measureJson(doc);
                if (need + 1 > cap)
                    return 0; // would not fit with NUL
                return serializeJson(doc, buf, cap);
            }

        } // namespace

        // =============================================================================
        // Public API
        // =============================================================================

        MsgType typeFromTopic(const char *topic)
        {
            if (!topic)
                return MsgType::UNKNOWN;
            struct Entry
            {
                const char *name;
                MsgType type;
            };
            static const Entry table[] = {
                {topic::CMD_DRIVE, MsgType::DRIVE},
                {topic::HB_CONSOLE, MsgType::HEARTBEAT},
                {topic::CMD_MODE, MsgType::MODE},
                {topic::CMD_ESTOP, MsgType::ESTOP},
                {topic::CMD_ESTOP_RESET, MsgType::ESTOP_RESET},
                {topic::CMD_START, MsgType::START},
                {topic::CMD_MISSION, MsgType::MISSION},
                {topic::CMD_GOTO, MsgType::GOTO},
                {topic::CMD_ARM, MsgType::ARM},
                {topic::CMD_POSE_SET, MsgType::POSE_SET},
                {topic::CMD_PING, MsgType::PING},
                {topic::CV_QR, MsgType::CV_QR},
                {topic::CV_COLOR, MsgType::CV_COLOR},
            };
            for (const Entry &e : table)
                if (strcmp(topic, e.name) == 0)
                    return e.type;
            return MsgType::UNKNOWN;
        }

        Sender senderOf(MsgType type)
        {
            return (type == MsgType::CV_QR || type == MsgType::CV_COLOR)
                       ? Sender::CV
                       : Sender::CONSOLE;
        }

        const char *errorName(DecodeError e)
        {
            switch (e)
            {
            case DecodeError::OK:
                return "OK";
            case DecodeError::BAD_JSON:
                return "BAD_JSON";
            case DecodeError::BAD_ENVELOPE:
                return "BAD_ENVELOPE";
            case DecodeError::MISSING_FIELD:
                return "MISSING_FIELD";
            case DecodeError::BAD_VALUE:
                return "BAD_VALUE";
            case DecodeError::UNKNOWN_TOPIC:
                return "UNKNOWN_TOPIC";
            }
            return "UNKNOWN";
        }

        const char *modeName(Mode m)
        {
            switch (m)
            {
            case Mode::MANUAL:
                return "MANUAL";
            case Mode::SEMI:
                return "SEMI";
            case Mode::AUTO:
                return "AUTO";
            case Mode::ESTOP:
                return "ESTOP";
            }
            return "UNKNOWN";
        }

        const char *armStatusName(ArmStatus s)
        {
            switch (s)
            {
            case ArmStatus::IDLE:
                return "IDLE";
            case ArmStatus::BUSY:
                return "BUSY";
            case ArmStatus::DONE:
                return "DONE";
            case ArmStatus::FAILED:
                return "FAILED";
            }
            return "UNKNOWN";
        }

        bool parseQrString(const char *s, float &x, float &y)
        {
            if (!s || s[0] != 'X' || s[1] != '=')
                return false;
            const char *p = s + 2;
            char *end = nullptr;
            float fx = strtof(p, &end);
            if (end == p)
                return false;
            if (end[0] != '&' || end[1] != 'Y' || end[2] != '=')
                return false;
            p = end + 3;
            float fy = strtof(p, &end);
            if (end == p || *end != '\0')
                return false;
            if (!std::isfinite(fx) || !std::isfinite(fy))
                return false;
            x = fx;
            y = fy;
            return true;
        }

        DecodeError decode(const char *topicName, const char *payload, size_t len,
                           Message &out)
        {
            out = Message();
            MsgType t = typeFromTopic(topicName);
            if (t == MsgType::UNKNOWN)
                return DecodeError::UNKNOWN_TOPIC;
            out.type = t;

            if (payload == nullptr || len == 0 || len > MAX_PAYLOAD)
                return DecodeError::BAD_JSON;

            JsonDocument doc;
            if (deserializeJson(doc, payload, len))
                return DecodeError::BAD_JSON;
            JsonObjectConst root = doc.as<JsonObjectConst>();
            if (root.isNull())
                return DecodeError::BAD_JSON;

            TRY(parseEnvelope(root, out.env));

            switch (t)
            {
            case MsgType::DRIVE:
                TRY(readFloat(root["thr"], -1.0f, 1.0f, out.thr));
                TRY(readFloat(root["turn"], -1.0f, 1.0f, out.turn));
                return DecodeError::OK;

            case MsgType::HEARTBEAT:
            case MsgType::ESTOP:
            case MsgType::ESTOP_RESET:
            case MsgType::START:
                return DecodeError::OK; // envelope only

            case MsgType::MODE:
            {
                const char *s = nullptr;
                TRY(readString(root["mode"], s));
                if (strcmp(s, "MANUAL") == 0)
                    out.mode = Mode::MANUAL;
                else if (strcmp(s, "SEMI") == 0)
                    out.mode = Mode::SEMI;
                else if (strcmp(s, "AUTO") == 0)
                    out.mode = Mode::AUTO;
                else
                    return DecodeError::BAD_VALUE; // ESTOP only via its topic
                return DecodeError::OK;
            }

            case MsgType::MISSION:
                return decodeMission(root, out.mission);

            case MsgType::GOTO:
                TRY(readFloat(root["x"], FIELD_MIN_M, FIELD_MAX_M, out.gotoX));
                TRY(readFloat(root["y"], FIELD_MIN_M, FIELD_MAX_M, out.gotoY));
                return DecodeError::OK;

            case MsgType::ARM:
            {
                const char *s = nullptr;
                TRY(readString(root["action"], s));
                if (strcmp(s, "PICK") == 0)
                    out.arm = ArmCmd::PICK;
                else if (strcmp(s, "DROP") == 0)
                    out.arm = ArmCmd::DROP;
                else if (strcmp(s, "STOW") == 0)
                    out.arm = ArmCmd::STOW;
                else
                    return DecodeError::BAD_VALUE;
                return DecodeError::OK;
            }

            case MsgType::POSE_SET:
                TRY(readFloat(root["x"], -0.5f, 3.5f, out.poseX));
                TRY(readFloat(root["y"], -0.5f, 3.5f, out.poseY));
                TRY(readFloat(root["theta"], -6.2832f, 6.2832f, out.poseTheta));
                return DecodeError::OK;

            case MsgType::PING:
            {
                long id = 0;
                TRY(readInt(root["ping_id"], 0, 2147483647L, id));
                out.pingId = (int32_t)id;
                return DecodeError::OK;
            }

            case MsgType::CV_QR:
                return decodeCvQr(root, out.env, out.qr);

            case MsgType::CV_COLOR:
            {
                TRY(readBool(root["valid"], out.colorValid));
                if (!out.colorValid)
                {
                    out.color = CvColor::NONE;
                    return DecodeError::OK;
                }
                const char *s = nullptr;
                TRY(readString(root["color"], s));
                if (strcmp(s, "BLUE") == 0)
                    out.color = CvColor::BLUE;
                else if (strcmp(s, "RED") == 0)
                    out.color = CvColor::RED;
                else if (strcmp(s, "NONE") == 0)
                    out.color = CvColor::NONE;
                else
                    return DecodeError::BAD_VALUE;
                return DecodeError::OK;
            }

            default:
                return DecodeError::UNKNOWN_TOPIC;
            }
        }

        // ---- Encoders ----------------------------------------------------------------

        size_t encodeTelemetry(char *buf, size_t cap, const Envelope &env, TimeMs t,
                               const Telemetry &tel)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            doc["x"] = round3(tel.pose.x);
            doc["y"] = round3(tel.pose.y);
            doc["theta"] = round3(tel.pose.theta);
            doc["v"] = round3(tel.pose.v);
            doc["w"] = round3(tel.pose.w);
            doc["mode"] = modeName(tel.mode);
            doc["auto_state"] = tel.autoState ? tel.autoState : "";
            doc["arm"] = armStatusName(tel.arm);
            if (std::isfinite(tel.battV))
                doc["batt_v"] = round3(tel.battV);
            else
                doc["batt_v"] = nullptr;
            doc["pose_valid"] = tel.pose.valid;
            doc["link_ok"] = tel.linkOk;
            return finish(doc, buf, cap);
        }

        size_t encodeModeState(char *buf, size_t cap, const Envelope &env, TimeMs t,
                               Mode m)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            doc["mode"] = modeName(m);
            return finish(doc, buf, cap);
        }

        size_t encodeHeartbeat(char *buf, size_t cap, const Envelope &env, TimeMs t)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            return finish(doc, buf, cap);
        }

        size_t encodePong(char *buf, size_t cap, const Envelope &env, TimeMs t,
                          int32_t pingId)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            doc["ping_id"] = pingId;
            return finish(doc, buf, cap);
        }

        size_t encodeEvent(char *buf, size_t cap, const Envelope &env, TimeMs t,
                           const char *ev, const char *detail)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            doc["ev"] = ev ? ev : "";
            doc["detail"] = detail ? detail : "";
            return finish(doc, buf, cap);
        }

        size_t encodeReject(char *buf, size_t cap, const Envelope &env, TimeMs t,
                            const char *rejectedTopic, DecodeError reason)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            doc["ev"] = "CMD_REJECTED";
            doc["topic"] = rejectedTopic ? rejectedTopic : "";
            doc["reason"] = errorName(reason);
            return finish(doc, buf, cap);
        }

        size_t encodeLog(char *buf, size_t cap, const Envelope &env, TimeMs t,
                         const char *level, const char *msg)
        {
            JsonDocument doc;
            stamp(doc, env, t);
            doc["lvl"] = level ? level : "INFO";
            doc["msg"] = msg ? msg : "";
            return finish(doc, buf, cap);
        }

    } // namespace proto
} // namespace robot
