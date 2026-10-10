// comms_task.cpp -- ESP32 only (empty in env:native).
#ifdef ARDUINO

#include "comms_task.h"

#include <Arduino.h>
#include <esp_system.h>
#include <stdio.h>

#include "mqtt_manager.h"

namespace robot {
namespace {

class MqttPublisher : public IPublisher {
public:
    bool publish(const char* topic, const char* payload, size_t len,
                 bool retain, bool critical) override {
        return MQTTManager::publish(topic, payload, len, retain, critical);
    }
};

MqttPublisher g_pub;
CommsCore*    g_core = nullptr;

// Runs in MQTT_Task. CommsCore::onMessage is short and never blocks.
void onRx(const char* topic, const char* payload, size_t len) {
    if (g_core) g_core->onMessage(topic, payload, len, millis());
}

void txTask(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        g_core->tick(millis(), MQTTManager::connectCount(),
                     MQTTManager::isConnected());
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

}  // namespace

bool startComms(ModeManager& modes, const CommsPorts& ports,
                const CommsSources& src, const CommsConfig& cfg) {
    char sid[8];
    snprintf(sid, sizeof sid, "r%06x", (unsigned)(esp_random() & 0xFFFFFF));
    g_core = new CommsCore(modes, ports, src, cfg, sid, &g_pub);

    if (!MQTTManager::begin(onRx)) return false;
    return xTaskCreatePinnedToCore(txTask, "Comms_TX", 4096, nullptr, 1,
                                   nullptr, 0) == pdPASS;
}

CommsCore& comms() { return *g_core; }

}  // namespace robot

#endif  // ARDUINO
