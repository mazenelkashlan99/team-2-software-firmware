// comms_task.cpp -- WiFi + MQTT link to the console.
#include "comms_task.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <atomic>
#include <string.h>

#include "secrets.h"     // WIFI_SSID, WIFI_PASS, MQTT_HOST, MQTT_PORT  (git-ignored)
#include "config.h"
#include "Topics.h"      // all topic strings
#include "Protocol.h"
#include "ModeManager.h"

#if defined(OTA_PASSWORD)
#define OTA_STRINGIFY_INNER(value) #value
#define OTA_STRINGIFY(value) OTA_STRINGIFY_INNER(value)
#endif

using namespace robot;

// Defined in main.cpp.
extern ModeManager g_modeManager;

QueueHandle_t g_driveQ = nullptr;
QueueHandle_t g_eventQ = nullptr;

namespace {

constexpr uint32_t WIFI_RETRY_MS   = 3000;
constexpr uint32_t MQTT_RETRY_MS   = 2000;
constexpr uint32_t LINK_TIMEOUT_MS = 500;   // 5 missed beats at 10 Hz
constexpr uint32_t TELEMETRY_MS    = 100;   // 10 Hz
constexpr uint32_t HEARTBEAT_MS    = 200;   // 5 Hz
constexpr size_t   MQTT_BUF        = 1280;  // > MAX_PAYLOAD + topic + header

WiFiClient   s_wifi;
PubSubClient s_client(s_wifi);

std::atomic<bool> s_linkOk{false};

uint32_t s_lastHbMs     = 0;
bool     s_haveHb       = false;
bool     s_otaStarted   = false;
uint32_t s_lastWifiTry  = 0;
uint32_t s_lastMqttTry  = 0;
uint32_t s_lastTelemMs  = 0;
uint32_t s_lastHbPubMs  = 0;
uint32_t s_seq          = 0;
uint32_t s_dropped      = 0;   // events lost because the queue was full

// Deferred publishes: things the callback must NOT do itself, because
// publishing from inside PubSubClient's callback re-enters loop().
bool                s_pingPending = false;
int32_t             s_pingId      = 0;
bool                s_rejPending  = false;
char                s_rejTopic[48] = {0};
proto::DecodeError  s_rejReason = proto::DecodeError::OK;

// ---- outgoing helpers --------------------------------------------------------------
proto::Envelope nextEnv() {
    proto::Envelope e;
    e.ver = proto::PROTOCOL_VERSION;
    strncpy(e.sid, ROBOT_SID, proto::SID_MAX - 1);
    e.seq = ++s_seq;
    return e;
}

void pub(const char* topicName, const char* buf, size_t n, bool retained = false) {
    if (n > 0 && s_client.connected())
        s_client.publish(topicName, (const uint8_t*)buf, (unsigned int)n, retained);
}

// ---- incoming: called by s_client.loop() once per message --------------------------
void onMqttMessage(char* topicName, uint8_t* payload, unsigned int len) {
    // 1. E-stop first: no decode, no queue. Hard override.
    if (strcmp(topicName, topic::CMD_ESTOP) == 0) {
        g_modeManager.triggerEstop(EstopReason::CONSOLE);
        return;
    }

    // 2. Everything else: decode, then dispatch.
    proto::Message msg;
    char jsonError[48] = {0};
    const proto::DecodeError err =
        proto::decode(topicName, (const char*)payload, len, msg,
                      jsonError, sizeof(jsonError));
    if (err != proto::DecodeError::OK) {
        if (err == proto::DecodeError::BAD_JSON) {
            Serial.printf("[mqtt] BAD_JSON topic=%s error=%s payload=",
                          topicName, jsonError[0] ? jsonError : "unknown");
            Serial.write(payload, len);
            Serial.println();
        } else {
            Serial.printf("[mqtt] rejected topic=%s reason=%s\n",
                          topicName, proto::errorName(err));
        }
        strncpy(s_rejTopic, topicName, sizeof(s_rejTopic) - 1);
        s_rejReason  = err;
        s_rejPending = true;
        return;
    }

    switch (msg.type) {
    case proto::MsgType::HEARTBEAT:
        s_lastHbMs = millis();
        s_haveHb   = true;
        break;

    case proto::MsgType::DRIVE: {
        DriveCmd c;
        c.thr = msg.thr;  // normalized [-1, 1]; scaled in control_task
        c.turn = msg.turn;
        xQueueOverwrite(g_driveQ, &c);   // keep only the newest
        break;
    }

    case proto::MsgType::PING:
        s_pingId      = msg.pingId;
        s_pingPending = true;
        break;

    default:
        // Everything else goes to events_task.
        if (xQueueSend(g_eventQ, &msg, 0) != pdTRUE) s_dropped++;
        break;
    }
}

// ---- connection management ---------------------------------------------------------
void ensureWifi(uint32_t now) {
    if (WiFi.status() == WL_CONNECTED) return;
    if (now - s_lastWifiTry < WIFI_RETRY_MS) return;
    s_lastWifiTry = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void ensureOta() {
    if (s_otaStarted || WiFi.status() != WL_CONNECTED) return;

#if defined(OTA_PASSWORD)
    ArduinoOTA.setPassword(OTA_STRINGIFY(OTA_PASSWORD));
#else
    // Keep OTA disabled unless an authentication password is configured.
    return;
#endif

    ArduinoOTA.setHostname(ROBOT_SID);
    ArduinoOTA.begin();
    s_otaStarted = true;
    Serial.printf("OTA ready at %s.local (%s)\n", ROBOT_SID,
                  WiFi.localIP().toString().c_str());
}

void ensureMqtt(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED || s_client.connected()) return;
    if (now - s_lastMqttTry < MQTT_RETRY_MS) return;
    s_lastMqttTry = now;

    // Last Will: broker publishes OFFLINE if the robot disappears.
    if (!s_client.connect(ROBOT_SID, nullptr, nullptr,
                          topic::STATUS_ONLINE, 1, true, topic::PAYLOAD_OFFLINE))
        return;

    s_client.subscribe(topic::CMD_DRIVE, 1);
    s_client.subscribe(topic::CMD_MODE, 1);
    s_client.subscribe(topic::CMD_ESTOP, 1);
    s_client.subscribe(topic::CMD_ESTOP_RESET, 1);
    s_client.subscribe(topic::CMD_START, 1);
    s_client.subscribe(topic::CMD_MISSION, 1);
    s_client.subscribe(topic::CMD_GOTO, 1);
    s_client.subscribe(topic::CMD_ARM, 1);
    s_client.subscribe(topic::CMD_POSE_SET, 1);
    s_client.subscribe(topic::CMD_PING, 0);
    s_client.subscribe(topic::CV_QR, 1);
    s_client.subscribe(topic::CV_COLOR, 1);
    s_client.subscribe(topic::HB_CONSOLE, 0);

    s_client.publish(topic::STATUS_ONLINE, topic::PAYLOAD_ONLINE, true);

    // Announce current mode right away so the console knows our state.
    char buf[128];
    pub(topic::MODE_STATE, buf,
        proto::encodeModeState(buf, sizeof(buf), nextEnv(), millis(),
                               g_modeManager.current()),
        true);
}

}  // namespace

// ---- public API --------------------------------------------------------------------
void commsInit() {
    g_driveQ = xQueueCreate(1, sizeof(Command));
    g_eventQ = xQueueCreate(8, sizeof(proto::Message));
}

bool commsLinkOk() { return s_linkOk.load(); }

void commsTask(void*) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                 // power-save adds latency to the link
    s_client.setServer(MQTT_HOST, MQTT_PORT);
    s_client.setBufferSize(MQTT_BUF);
    s_client.setKeepAlive(5);
    s_client.setSocketTimeout(2);         // default 15 s would stall the task
    s_client.setCallback(onMqttMessage);

    TickType_t last = xTaskGetTickCount();

    for (;;) {
        const uint32_t now = millis();

        ensureWifi(now);
        ensureOta();
        if (s_otaStarted) ArduinoOTA.handle();
        ensureMqtt(now);
        if (s_client.connected()) s_client.loop();

        // Link watchdog: fresh console heartbeat AND broker connected.
        const bool ok = s_haveHb
                     && s_client.connected()
                     && (now - s_lastHbMs) < LINK_TIMEOUT_MS;
        if (s_linkOk.load() && !ok) {
            g_modeManager.triggerEstop(EstopReason::WATCHDOG);
        }
        s_linkOk.store(ok);

        if (s_client.connected()) {
            char buf[512];

            // Replies the callback could not send itself.
            if (s_pingPending) {
                s_pingPending = false;
                pub(topic::PONG, buf,
                    proto::encodePong(buf, sizeof(buf), nextEnv(), now, s_pingId));
            }
            if (s_rejPending) {
                s_rejPending = false;
                pub(topic::EVENT, buf,
                    proto::encodeReject(buf, sizeof(buf), nextEnv(), now,
                                        s_rejTopic, s_rejReason));
            }

            // Periodic telemetry.
            if (now - s_lastTelemMs >= TELEMETRY_MS) {
                s_lastTelemMs = now;
                proto::Telemetry t {};
                comms_fill_telemetry(t);
                t.mode   = g_modeManager.current();
                t.linkOk = ok;
                pub(topic::TELEM, buf,
                    proto::encodeTelemetry(buf, sizeof(buf), nextEnv(), now, t));
            }

            // Robot-side heartbeat.
            if (now - s_lastHbPubMs >= HEARTBEAT_MS) {
                s_lastHbPubMs = now;
                pub(topic::HB_ROBOT, buf,
                    proto::encodeHeartbeat(buf, sizeof(buf), nextEnv(), now));
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}
