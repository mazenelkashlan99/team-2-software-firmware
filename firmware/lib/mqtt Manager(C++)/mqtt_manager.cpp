// mqtt_manager.cpp -- see mqtt_manager.h
#include "mqtt_manager.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <atomic>

#include "secrets.h"      // WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_HOST/PORT
#include "Topics.h"

using namespace robot;

// ================= Configuration =================

static const char*    MQTT_CLIENT_ID    = "robot_esp32";
static const uint16_t MQTT_KEEPALIVE_S  = 5;      // broker fires the LWT in ~7 s
static const uint16_t MQTT_RX_BUFFER    = 1280;   // > 1024 max payload + topic

static const uint32_t WIFI_RESTART_MS   = 15000;  // re-issue WiFi.begin if stuck
static const uint32_t MQTT_BACKOFF_MIN  = 500;
static const uint32_t MQTT_BACKOFF_MAX  = 5000;
static const uint32_t TASK_TICK_MS      = 2;      // RX latency <= ~2 ms
static const uint8_t  TX_QUEUE_DEPTH    = 16;

// What the robot listens to. QoS 1 for the commands that must not be lost
// while connected; the 30 Hz / 10 Hz streams stay at QoS 0.
struct Sub { const char* topic; uint8_t qos; };
static const Sub SUBS[] = {
    { topic::CMD_DRIVE,       0 },
    { topic::HB_CONSOLE,      0 },
    { topic::CMD_PING,        0 },
    { topic::CMD_ESTOP,       1 },
    { topic::CMD_ESTOP_RESET, 1 },
    { topic::CMD_ARM,         1 },
};
static const size_t SUB_COUNT = sizeof(SUBS) / sizeof(SUBS[0]);

// ================= State =================

static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);

static MqttRxCallback rxCallback = nullptr;

struct OutMsg {
    char     topic[MQTTManager::MAX_TOPIC];
    char     payload[MQTTManager::MAX_PAYLOAD];
    uint16_t len;
    bool     retain;
    bool     critical;
};

static QueueHandle_t txQueue = nullptr;

static std::atomic<bool>     connectedFlag(false);
static std::atomic<uint32_t> connectCounter(0);

// ================= Incoming =================

// Runs inside mqtt.loop(), in MQTT_Task. Hand the raw bytes to the protocol
// layer; do not parse here.
static void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    if (rxCallback) rxCallback(topic, (const char*)payload, length);
}

// ================= Connection management =================

static bool connectMqtt() {
    // Last Will: the broker publishes "offline" (retained) if we vanish.
    if (!mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr,
                      topic::STATUS_ONLINE, 1, true, topic::PAYLOAD_OFFLINE))
        return false;

    for (size_t i = 0; i < SUB_COUNT; i++) {
        if (!mqtt.subscribe(SUBS[i].topic, SUBS[i].qos)) {
            mqtt.disconnect();
            return false;
        }
    }
    mqtt.publish(topic::STATUS_ONLINE, (const uint8_t*)topic::PAYLOAD_ONLINE,
                 strlen(topic::PAYLOAD_ONLINE), true);
    return true;
}

// After a disconnect: throw away stale telemetry/heartbeat, keep critical ones.
static void dropNonCritical() {
    OutMsg m;
    UBaseType_t n = uxQueueMessagesWaiting(txQueue);
    while (n--) {
        if (xQueueReceive(txQueue, &m, 0) != pdTRUE) break;
        if (m.critical) xQueueSend(txQueue, &m, 0);
    }
}

static void flushTx() {
    OutMsg m;
    while (xQueueReceive(txQueue, &m, 0) == pdTRUE) {
        if (!mqtt.publish(m.topic, (const uint8_t*)m.payload, m.len, m.retain)) {
            // Socket write failed: the connection is probably gone.
            // Keep important messages, drop the rest.
            if (m.critical) xQueueSendToFront(txQueue, &m, 0);
            break;
        }
    }
}

static void mqttTask(void*) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);            // modem sleep adds 100+ ms of latency
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t wifiBeginMs = millis();
    uint32_t nextMqttTry = 0;
    uint32_t backoff     = MQTT_BACKOFF_MIN;

    for (;;) {
        const uint32_t now = millis();

        if (WiFi.status() != WL_CONNECTED) {
            if (connectedFlag.exchange(false)) dropNonCritical();
            if ((uint32_t)(now - wifiBeginMs) >= WIFI_RESTART_MS) {
                WiFi.disconnect();
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
                wifiBeginMs = now;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        wifiBeginMs = now;   // healthy: restart the "stuck" timer

        if (!mqtt.connected()) {
            if (connectedFlag.exchange(false)) dropNonCritical();
            if ((int32_t)(now - nextMqttTry) >= 0) {
                if (connectMqtt()) {
                    backoff = MQTT_BACKOFF_MIN;
                    connectCounter.fetch_add(1);
                    connectedFlag.store(true);
                } else {
                    nextMqttTry = now + backoff;
                    backoff = (backoff * 2 > MQTT_BACKOFF_MAX)
                                  ? MQTT_BACKOFF_MAX : backoff * 2;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        mqtt.loop();     // RX callbacks + keepalive
        flushTx();
        vTaskDelay(pdMS_TO_TICKS(TASK_TICK_MS));
    }
}

// ================= Public interface =================

bool MQTTManager::begin(MqttRxCallback callback) {
    rxCallback = callback;

    txQueue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(OutMsg));
    if (!txQueue) return false;

    mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    mqtt.setCallback(onMqttMessage);
    mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    mqtt.setSocketTimeout(1);
    if (!mqtt.setBufferSize(MQTT_RX_BUFFER)) return false;

    // Core 0 (Wi-Fi core). Control and pose tasks live on core 1.
    return xTaskCreatePinnedToCore(mqttTask, "MQTT_Task", 8192, nullptr, 2,
                                   nullptr, 0) == pdPASS;
}

bool MQTTManager::publish(const char* topic, const char* payload, size_t len,
                          bool retain, bool critical) {
    if (!txQueue || !topic || !payload) return false;
    if (len == 0 || len >= MAX_PAYLOAD) return false;
    if (strlen(topic) >= MAX_TOPIC) return false;
    if (!critical && !connectedFlag.load()) return false;

    OutMsg m;
    strcpy(m.topic, topic);
    memcpy(m.payload, payload, len);
    m.len = (uint16_t)len;
    m.retain = retain;
    m.critical = critical;
    return xQueueSend(txQueue, &m, 0) == pdTRUE;   // timeout 0: never blocks
}

bool MQTTManager::isConnected() { return connectedFlag.load(); }

uint32_t MQTTManager::connectCount() { return connectCounter.load(); }
