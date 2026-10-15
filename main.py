"""Run a local, broker-free smoke check of the supported MQTT commands."""

import json

from src.rov_mqtt.mqtt_client import MQTTClient


def main():
    client = MQTTClient(client_id="protocol_smoke_test")
    published = []
    client.client.publish = lambda topic, payload, qos=0: published.append(
        (topic, json.loads(payload), qos)
    )

    client.publish_drive(0.5, -0.2)
    client.publish_arm("STOW")
    client.publish_estop()
    client.publish_estop_reset()
    client.publish_ping(1)
    client.publish_console_heartbeat({"ts": 1})

    for topic, payload, qos in published:
        print(f"{topic} qos={qos}: {json.dumps(payload)}")


if __name__ == "__main__":
    main()
