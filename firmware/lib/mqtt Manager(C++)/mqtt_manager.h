// mqtt_manager.h -- MQTT transport for the ESP32 (PubSubClient + FreeRTOS).
//
// ESP32 only (not built in env:native).
//
// Rules this module keeps:
//   * begin() returns immediately; it never waits for Wi-Fi or the broker.
//     The robot boots (in ESTOP) with or without a network.
//   * publish() NEVER blocks: it copies into a queue and returns false if the
//     queue is full or the link is down.
//   * ALL PubSubClient calls happen in one task ("MQTT_Task", core 0), because
//     PubSubClient is not thread-safe.
//   * The receive callback runs inside that task: keep it SHORT (decode, set an
//     atomic, post to a queue). No delay(), no Serial printing per message.
//
// PubSubClient limits: it publishes at QoS 0 only (retain is supported). It
// can SUBSCRIBE at QoS 1 and acks incoming QoS 1 messages, which is what we
// use for estop / estop_reset / arm.
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// Called from MQTT_Task. payload is NOT guaranteed NUL-terminated; use len.
typedef void (*MqttRxCallback)(const char* topic, const char* payload,
                               size_t len);

class MQTTManager {
public:
    // Creates the queue and the task. Returns false only if memory is missing.
    static bool begin(MqttRxCallback callback);

    // Queue a message (non-blocking). payload/len are copied.
    //   retain   : MQTT retain flag.
    //   critical : keep and retry it if the socket write fails, and keep it
    //              across a reconnect (use for mode / events). Non-critical
    //              messages (telemetry, heartbeat) are dropped instead.
    // Returns true if queued. Non-critical messages are refused while the
    // broker link is down.
    static bool publish(const char* topic, const char* payload, size_t len,
                        bool retain = false, bool critical = false);

    static bool     isConnected();
    // Increments on every successful connect + subscribe.
    static uint32_t connectCount();

    static const size_t MAX_TOPIC   = 48;
    static const size_t MAX_PAYLOAD = 512;   // robot -> console messages
};
