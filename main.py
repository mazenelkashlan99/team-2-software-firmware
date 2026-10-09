import time

from src.rov_mqtt.mqtt_client import MQTTClient


def on_message(topic: str, payload: str) -> None:
    print(f"[RECEIVED] {topic} -> {payload}")


def main() -> None:
    mqtt_client = MQTTClient()
    mqtt_client.set_message_callback(on_message)

    try:
        # Connect to Mosquitto
        mqtt_client.connect()
        time.sleep(1)

        print("\n--- Testing MQTT Publish ---")

        # Safe drive test: zero throttle and zero turning
        mqtt_client.publish_drive(
            thr=0.0,
            turn=0.0,
        )

        # Test mode command
        mqtt_client.publish_mode("MANUAL")

        # Test ping command
        mqtt_client.publish_ping(
            ping_id=1,
            t_send=time.time(),
        )

        print("Test messages sent.")
        print("Listening for incoming messages for 5 seconds...")

        # Allow time to receive incoming messages
        time.sleep(5)

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")

    finally:
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()