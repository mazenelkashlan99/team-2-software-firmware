import paho.mqtt.client as mqtt
from typing import Callable
from paho.mqtt.client import CallbackAPIVersion


class MQTTClient:
    def __init__(
            self,
            broker_host: str = "localhost",      #Address of broker
            broker_port: str = 1883,
            client_id: str = "rov_software",
                ):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2,
                                  client_id=client_id,)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self.message_callback: Callable | None = None

    def connect(self) -> None:                      #handeling the broker connection
        self.client.connect(
            self.broker_host,
            self.broker_port,
        )
        self.client.loop_start()                   #turning network loop in the background thread

    def disconnect(self) -> None:
        self.client.loop_stop()
        self.client.disconnect()

    def publish(self,topic : str ,payload: str,qos : int = 0 ) -> None:
        self.client.publish(topic,payload,qos=qos)

    def subscribe(self,topic:str,qos:int = 0)-> None:
        self.client.subscribe(
            topic,
            qos=qos,
        )


    def set_message_callback(self,callback: Callable) -> None:
        self.message_callback = callback

    def _on_connect(self,client,userdata,flags,reason_code,properties,) -> None:
        print(f"Connected to MQTT broker: {reason_code}")

    def _on_disconnect(self,client,userdata,flags,reason_code,properties,) -> None:
        print(f"DisConnected to MQTT broker: {reason_code}")

    def _on_message(self,client,userdata,message,)->None:
        payload = message.payload.decode("utf-8")
        if self.message_callback:
            self.message_callback(
                message.topic,
                payload,
            )