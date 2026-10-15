#include <unity.h>
#include <cstring>
#include <cmath>
#include "Protocol.h"

using namespace robot;
using namespace robot::proto;

void setUp() {}
void tearDown() {}

// ---- parseQrString ---------------------------------------------------------

// The exact string from the rulebook must parse into the expected floats.
void test_qr_parse_rulebook_string() {
    float x = 0.0f, y = 0.0f;
    const bool ok = parseQrString("X=1.56&Y=0.49", x, y);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.56f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.49f, y);
}

// Junk must be rejected, not silently parsed as 0.
void test_qr_parse_rejects_garbage() {
    float x = 0.0f, y = 0.0f;
    TEST_ASSERT_FALSE(parseQrString(nullptr, x, y));
    TEST_ASSERT_FALSE(parseQrString("", x, y));
    TEST_ASSERT_FALSE(parseQrString("hello", x, y));
    TEST_ASSERT_FALSE(parseQrString("X=abc&Y=0.49", x, y));
    TEST_ASSERT_FALSE(parseQrString("X=1.56", x, y));         // missing Y
    TEST_ASSERT_FALSE(parseQrString("X=1.56&Y=", x, y));      // missing Y value
    TEST_ASSERT_FALSE(parseQrString("X=1.56&Y=0.49junk", x, y));
}

// Negative numbers should parse (a valid QR could be to the left of start).
void test_qr_parse_negative() {
    float x = 0.0f, y = 0.0f;
    TEST_ASSERT_TRUE(parseQrString("X=-0.5&Y=-0.25", x, y));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.5f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.25f, y);
}

// ---- typeFromTopic ---------------------------------------------------------

void test_type_from_topic_known() {
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::DRIVE),
                      static_cast<int>(typeFromTopic(topic::CMD_DRIVE)));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::MODE),
                      static_cast<int>(typeFromTopic(topic::CMD_MODE)));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::ESTOP),
                      static_cast<int>(typeFromTopic(topic::CMD_ESTOP)));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::ESTOP_RESET),
                      static_cast<int>(typeFromTopic(topic::CMD_ESTOP_RESET)));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::CV_QR),
                      static_cast<int>(typeFromTopic(topic::CV_QR)));
}

void test_type_from_topic_unknown() {
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::UNKNOWN),
                      static_cast<int>(typeFromTopic("robot/nope")));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::UNKNOWN),
                      static_cast<int>(typeFromTopic(nullptr)));
}

// ---- senderOf --------------------------------------------------------------

void test_sender_of_cv() {
    TEST_ASSERT_EQUAL(static_cast<int>(Sender::CV),
                      static_cast<int>(senderOf(MsgType::CV_QR)));
    TEST_ASSERT_EQUAL(static_cast<int>(Sender::CV),
                      static_cast<int>(senderOf(MsgType::CV_COLOR)));
    TEST_ASSERT_EQUAL(static_cast<int>(Sender::CONSOLE),
                      static_cast<int>(senderOf(MsgType::DRIVE)));
}

// ---- decode: envelope ------------------------------------------------------

// A valid DRIVE payload must decode successfully.
void test_decode_drive_ok() {
    const char* payload =
        R"({"ver":1,"sid":"gui-01","seq":42,"thr":0.75,"turn":-0.25})";
    Message m {};
    const DecodeError e = decode(topic::CMD_DRIVE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::OK), static_cast<int>(e));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::DRIVE),
                      static_cast<int>(m.type));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.75f, m.thr);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.25f, m.turn);
    TEST_ASSERT_EQUAL_UINT32(42, m.env.seq);
    TEST_ASSERT_EQUAL_STRING("gui-01", m.env.sid);
}

// Malformed JSON -> BAD_JSON.
void test_decode_bad_json() {
    const char* payload = R"({"ver":1, "sid":)";
    Message m {};
    const DecodeError e = decode(topic::CMD_DRIVE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_JSON),
                      static_cast<int>(e));
}

// Wrong protocol version -> BAD_ENVELOPE.
void test_decode_bad_version() {
    const char* payload =
        R"({"ver":99,"sid":"gui-01","seq":1,"thr":0.0,"turn":0.0})";
    Message m {};
    const DecodeError e = decode(topic::CMD_DRIVE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_ENVELOPE),
                      static_cast<int>(e));
}

// Missing sid -> BAD_ENVELOPE.
void test_decode_missing_sid() {
    const char* payload = R"({"ver":1,"seq":1,"thr":0.0,"turn":0.0})";
    Message m {};
    const DecodeError e = decode(topic::CMD_DRIVE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_ENVELOPE),
                      static_cast<int>(e));
}

// ---- decode: value checks --------------------------------------------------

// thr outside [-1, 1] -> BAD_VALUE, not accepted-and-clamped.
void test_decode_drive_thr_out_of_range() {
    const char* payload =
        R"({"ver":1,"sid":"gui-01","seq":1,"thr":1.5,"turn":0.0})";
    Message m {};
    const DecodeError e = decode(topic::CMD_DRIVE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_VALUE),
                      static_cast<int>(e));
}

// Missing thr -> MISSING_FIELD.
void test_decode_drive_missing_field() {
    const char* payload = R"({"ver":1,"sid":"gui-01","seq":1,"turn":0.0})";
    Message m {};
    const DecodeError e = decode(topic::CMD_DRIVE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::MISSING_FIELD),
                      static_cast<int>(e));
}

// ---- decode: MODE ----------------------------------------------------------

void test_decode_mode_ok() {
    const char* payload = R"({"ver":1,"sid":"gui","seq":7,"mode":"AUTO"})";
    Message m {};
    const DecodeError e = decode(topic::CMD_MODE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::OK), static_cast<int>(e));
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::AUTO),
                      static_cast<int>(m.mode));
}

// ESTOP must NOT be settable via the MODE topic — only via CMD_ESTOP.
void test_decode_mode_rejects_estop() {
    const char* payload = R"({"ver":1,"sid":"gui","seq":7,"mode":"ESTOP"})";
    Message m {};
    const DecodeError e = decode(topic::CMD_MODE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_VALUE),
                      static_cast<int>(e));
}

// Unknown mode string -> BAD_VALUE.
void test_decode_mode_bad_string() {
    const char* payload = R"({"ver":1,"sid":"gui","seq":7,"mode":"TURBO"})";
    Message m {};
    const DecodeError e = decode(topic::CMD_MODE, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_VALUE),
                      static_cast<int>(e));
}

// ---- decode: ESTOP is a no-payload command ---------------------------------

void test_decode_estop_no_payload() {
    Message m {};
    const DecodeError e = decode(topic::CMD_ESTOP, nullptr, 0, m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::OK), static_cast<int>(e));
    TEST_ASSERT_EQUAL(static_cast<int>(MsgType::ESTOP),
                      static_cast<int>(m.type));
}

// ---- decode: CV_QR ---------------------------------------------------------

void test_decode_cv_qr_ok() {
    const char* payload =
        R"({"ver":1,"sid":"cv","seq":9,"valid":true,"raw":"X=1.56&Y=0.49",)"
        R"("x":1.56,"y":0.49})";
    Message m {};
    const DecodeError e = decode(topic::CV_QR, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::OK), static_cast<int>(e));
    TEST_ASSERT_TRUE(m.qr.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1.56f, m.qr.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.49f, m.qr.y);
    TEST_ASSERT_EQUAL_UINT32(9, m.qr.seq);
}

// "no QR visible" is a valid, non-error state.
void test_decode_cv_qr_invalid_is_ok() {
    const char* payload =
        R"({"ver":1,"sid":"cv","seq":10,"valid":false})";
    Message m {};
    const DecodeError e = decode(topic::CV_QR, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::OK), static_cast<int>(e));
    TEST_ASSERT_FALSE(m.qr.valid);
}

// CV's parsed numbers disagreeing with the raw string by >1 cm -> BAD_VALUE.
void test_decode_cv_qr_numbers_disagree() {
    const char* payload =
        R"({"ver":1,"sid":"cv","seq":9,"valid":true,"raw":"X=1.56&Y=0.49",)"
        R"("x":1.99,"y":0.49})";
    Message m {};
    const DecodeError e = decode(topic::CV_QR, payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_VALUE),
                      static_cast<int>(e));
}

// ---- decode: MISSION -------------------------------------------------------

void test_decode_mission_ok() {
    const char* payload =
        R"({"ver":1,"sid":"gui","seq":1,"boxes":[
            {"id":0,"type":"BLUE","x":0.5,"y":3.0},
            {"id":1,"type":"RED","x":1.5,"y":3.0}
           ],"zones":[
            {"type":"BLUE","x":0.5,"y":0.5}
           ]})";
    Message m {};
    const DecodeError e = decode(topic::CMD_MISSION, payload,
                                 strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::OK), static_cast<int>(e));
    TEST_ASSERT_EQUAL_UINT8(2, m.mission.boxCount);
    TEST_ASSERT_EQUAL_UINT8(1, m.mission.zoneCount);
    TEST_ASSERT_EQUAL(static_cast<int>(BoxType::BLUE_EASY),
                      static_cast<int>(m.mission.boxes[0].type));
    TEST_ASSERT_EQUAL(static_cast<int>(BoxType::RED_HARD),
                      static_cast<int>(m.mission.boxes[1].type));
}

// A zone with type QR is invalid — QR boxes have no zone.
void test_decode_mission_rejects_qr_zone() {
    const char* payload =
        R"({"ver":1,"sid":"gui","seq":1,"boxes":[
            {"id":0,"type":"BLUE","x":0.5,"y":3.0}
           ],"zones":[
            {"type":"QR","x":0.5,"y":0.5}
           ]})";
    Message m {};
    const DecodeError e = decode(topic::CMD_MISSION, payload,
                                 strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_VALUE),
                      static_cast<int>(e));
}

// Too many boxes -> BAD_VALUE (MAX_BOXES is 8).
void test_decode_mission_too_many_boxes() {
    // Hand-write 9 boxes; that must exceed MAX_BOXES.
    const char* payload =
        R"({"ver":1,"sid":"gui","seq":1,"zones":[],"boxes":[
            {"id":0,"type":"BLUE","x":0.5,"y":3.0},
            {"id":1,"type":"BLUE","x":0.6,"y":3.0},
            {"id":2,"type":"BLUE","x":0.7,"y":3.0},
            {"id":3,"type":"BLUE","x":0.8,"y":3.0},
            {"id":4,"type":"BLUE","x":0.9,"y":3.0},
            {"id":5,"type":"BLUE","x":1.0,"y":3.0},
            {"id":6,"type":"BLUE","x":1.1,"y":3.0},
            {"id":7,"type":"BLUE","x":1.2,"y":3.0},
            {"id":8,"type":"BLUE","x":1.3,"y":3.0}
           ]})";
    Message m {};
    const DecodeError e = decode(topic::CMD_MISSION, payload,
                                 strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::BAD_VALUE),
                      static_cast<int>(e));
}

// ---- decode: unknown topic -------------------------------------------------

void test_decode_unknown_topic() {
    const char* payload = R"({"ver":1,"sid":"gui","seq":1})";
    Message m {};
    const DecodeError e = decode("robot/bogus", payload, strlen(payload), m);
    TEST_ASSERT_EQUAL(static_cast<int>(DecodeError::UNKNOWN_TOPIC),
                      static_cast<int>(e));
}

// ---- encode ----------------------------------------------------------------

// encodeHeartbeat must produce JSON with the envelope and a "t" field.
void test_encode_heartbeat() {
    Envelope env {};
    strncpy(env.sid, "robot-01", SID_MAX - 1);
    env.seq = 100;

    char buf[256];
    const size_t n = encodeHeartbeat(buf, sizeof(buf), env, 1234);
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(n < sizeof(buf));

    // Quick-and-dirty content check — not a full parse, just a smell test.
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"ver\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"sid\":\"robot-01\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"seq\":100"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"t\":1234"));
}

// encodeModeState embeds the human-readable mode name.
void test_encode_mode_state() {
    Envelope env {};
    strncpy(env.sid, "robot-01", SID_MAX - 1);

    char buf[256];
    const size_t n = encodeModeState(buf, sizeof(buf), env, 0, Mode::SEMI);
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"mode\":\"SEMI\""));
}

// encodeReject embeds the topic and the error name string.
void test_encode_reject() {
    Envelope env {};
    strncpy(env.sid, "robot-01", SID_MAX - 1);

    char buf[256];
    const size_t n = encodeReject(buf, sizeof(buf), env, 0,
                                  topic::CMD_DRIVE, DecodeError::BAD_VALUE);
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"ev\":\"CMD_REJECTED\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"reason\":\"BAD_VALUE\""));
}

// encodePong embeds the ping id.
void test_encode_pong() {
    Envelope env {};
    strncpy(env.sid, "robot-01", SID_MAX - 1);

    char buf[256];
    const size_t n = encodePong(buf, sizeof(buf), env, 0, 31415);
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"ping_id\":31415"));
}

// Encoders must fail cleanly when the buffer is too small.
void test_encode_buffer_too_small() {
    Envelope env {};
    char tiny[4];
    const size_t n = encodeHeartbeat(tiny, sizeof(tiny), env, 0);
    TEST_ASSERT_EQUAL_UINT32(0, n);
}

// ---- Names -----------------------------------------------------------------

void test_mode_names() {
    TEST_ASSERT_EQUAL_STRING("MANUAL", modeName(Mode::MANUAL));
    TEST_ASSERT_EQUAL_STRING("SEMI",   modeName(Mode::SEMI));
    TEST_ASSERT_EQUAL_STRING("AUTO",   modeName(Mode::AUTO));
    TEST_ASSERT_EQUAL_STRING("ESTOP",  modeName(Mode::ESTOP));
}

void test_error_names() {
    TEST_ASSERT_EQUAL_STRING("OK",            errorName(DecodeError::OK));
    TEST_ASSERT_EQUAL_STRING("BAD_JSON",      errorName(DecodeError::BAD_JSON));
    TEST_ASSERT_EQUAL_STRING("BAD_ENVELOPE",  errorName(DecodeError::BAD_ENVELOPE));
    TEST_ASSERT_EQUAL_STRING("MISSING_FIELD", errorName(DecodeError::MISSING_FIELD));
    TEST_ASSERT_EQUAL_STRING("BAD_VALUE",     errorName(DecodeError::BAD_VALUE));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN_TOPIC", errorName(DecodeError::UNKNOWN_TOPIC));
}

// ---- Runner ----------------------------------------------------------------

int main(int, char**) {
    UNITY_BEGIN();

    RUN_TEST(test_qr_parse_rulebook_string);
    RUN_TEST(test_qr_parse_rejects_garbage);
    RUN_TEST(test_qr_parse_negative);

    RUN_TEST(test_type_from_topic_known);
    RUN_TEST(test_type_from_topic_unknown);
    RUN_TEST(test_sender_of_cv);

    RUN_TEST(test_decode_drive_ok);
    RUN_TEST(test_decode_bad_json);
    RUN_TEST(test_decode_bad_version);
    RUN_TEST(test_decode_missing_sid);
    RUN_TEST(test_decode_drive_thr_out_of_range);
    RUN_TEST(test_decode_drive_missing_field);

    RUN_TEST(test_decode_mode_ok);
    RUN_TEST(test_decode_mode_rejects_estop);
    RUN_TEST(test_decode_mode_bad_string);

    RUN_TEST(test_decode_estop_no_payload);

    RUN_TEST(test_decode_cv_qr_ok);
    RUN_TEST(test_decode_cv_qr_invalid_is_ok);
    RUN_TEST(test_decode_cv_qr_numbers_disagree);

    RUN_TEST(test_decode_mission_ok);
    RUN_TEST(test_decode_mission_rejects_qr_zone);
    RUN_TEST(test_decode_mission_too_many_boxes);

    RUN_TEST(test_decode_unknown_topic);

    RUN_TEST(test_encode_heartbeat);
    RUN_TEST(test_encode_mode_state);
    RUN_TEST(test_encode_reject);
    RUN_TEST(test_encode_pong);
    RUN_TEST(test_encode_buffer_too_small);

    RUN_TEST(test_mode_names);
    RUN_TEST(test_error_names);

    return UNITY_END();
}