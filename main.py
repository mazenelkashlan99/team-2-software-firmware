from rov_mqtt.mqtt_client import MQTTClient

def on_message(topic: str, payload: str) -> None:
    print(f"[RECEIVED] {topic} -> {payload}")


mqtt_client = MQTTClient()

mqtt_client.set_message_callback(on_message)
mqtt_client.connect()

print("MQTT client is running. Press Ctrl+C to stop.")

try:
    while True:
        pass
except KeyboardInterrupt:
    print("\nStopping MQTT client...")
    mqtt_client.disconnect()