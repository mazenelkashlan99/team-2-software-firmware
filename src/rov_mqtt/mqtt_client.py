import paho.mqtt.client as mqtt
from typing import Callable
from paho.mqtt.client import CallbackAPIVersion
import json
from threading import Event, Thread ,Lock
import time
from uuid import uuid4
import math
import os

class MQTTClient:
    def __init__(
            self,
            broker_host: str | None = None,      #Address of broker
            broker_port: int = 1883,
            client_id: str = "rov_software",
            heartbeat_envelope_factory: Callable[[], dict[str, object]] | None = None,
                ):
        self.broker_host = (
            broker_host
            if broker_host is not None
            else os.environ.get("ROV_MQTT_BROKER_HOST", "localhost")
        )
        self.broker_port = broker_port

        self.sid = uuid4().hex[:15]
        self._message_seq = 0 # Sequence number for heartbeat messages
        self._sequence_lock = Lock()  # Lock for sequence number updates

        self._heartbeat_stop = Event()
        self._heartbeat_thread: Thread | None = None
        self._heartbeat_envelope_factory=( 
            heartbeat_envelope_factory
            if heartbeat_envelope_factory is not None
            else self._create_heartbeat_envelope)
        self.client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2,
                                  client_id=client_id,)
        self.client.on_connect = self._on_connect
        self.client.on_subscribe = self._on_subscribe
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self.message_callback: Callable | None = None
        self.connection_callback: Callable[[bool], None] | None = None


        



    def _create_heartbeat_envelope(self)-> dict [str,object]:
        return{ "ts": int(time.time()*1000),}
    def connect(self) -> None:                      #handeling the broker connection
        print(
            f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port}"
        )
        self.client.connect_async(
            self.broker_host,
            self.broker_port,
        )
        self.client.loop_start()                   #turning network loop in the background thread

    def disconnect(self) -> None:
        self.stop_console_heartbeat()
        self.client.disconnect()
        self.client.loop_stop()

    def publish(self,topic : str ,payload: str,qos : int = 0 ):
        try:
            message_data = json.loads(payload)
        except json.JSONDecodeError:
            message_data={"data": payload}

        if not isinstance(message_data,dict):
            message_data ={"data": payload}

        with self._sequence_lock:
            self._message_seq +=1
            sequence = self._message_seq

        message_data["ver"]=1
        message_data["sid"] = self.sid
        message_data["seq"] = sequence

        payload_json = json.dumps(message_data)
        return self.client.publish(topic,payload_json,qos=qos)
        

    def subscribe(self,topic:str,qos:int = 0)-> None:
        self.client.subscribe(
            topic,
            qos=qos,
        )


    def set_message_callback(self,callback: Callable) -> None:
        self.message_callback = callback

    def set_connection_callback(self, callback: Callable[[bool], None]) -> None:
        self.connection_callback = callback

    def _on_connect(self,client,userdata,flags,reason_code,properties,) -> None:
        print(
            f"Connected to MQTT broker {self.broker_host}:{self.broker_port}: "
            f"{reason_code}"
        )
        if reason_code != 0:
            return
        self.subscribe_telemetry()
        self.subscribe_state_mode()
        self.subscribe_status_online()
        self.subscribe_robot_heartbeat()
        self.subscribe_pong()
        self.subscribe_log()
        if self._heartbeat_envelope_factory is not None:
            self.start_console_heartbeat(
                self._heartbeat_envelope_factory,rate_hz=10.0,
            )
        if self.connection_callback:
            self.connection_callback(True)
        print("All subscriptions requested.")

    def _on_subscribe(self, client, userdata, mid, reason_code_list, properties) -> None:
        print(f"MQTT subscription acknowledged (mid={mid}): {reason_code_list}")

    def _on_disconnect(self,client,userdata,flags,reason_code,properties,) -> None:
        print(f"DisConnected to MQTT broker: {reason_code}")
        self.stop_console_heartbeat()
        if self.connection_callback:
            self.connection_callback(False)


    def _on_message(self,client,userdata,message,)->None:
        payload = message.payload.decode("utf-8", errors="replace")
        if self.message_callback:
            try:
                self.message_callback(message.topic, payload)
            except Exception as error:
                print(f"MQTT message callback failed for {message.topic}: {error}")

    def publish_drive(self,thr:float,turn:float) -> None:
        if not self._is_finite_number(thr) or not -1 <= thr <= 1:
            raise ValueError("thr must be between -1 and 1")
        if not self._is_finite_number(turn) or not -1 <= turn <= 1:
            raise ValueError("turn must be between -1 and 1")

        payload ={
            "thr":thr,
            "turn":turn,
        }
        payload_json=json.dumps(payload)

        self.publish(topic="robot/cmd/drive",
                     payload=payload_json,
                     qos=0,                     #because  messages comes after miliseconds
                     )

    @staticmethod
    def _is_finite_number(value: object) -> bool:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
        try:
            return math.isfinite(float(value))
        except (OverflowError, ValueError):
            return False

    def publish_estop(self,reason:str | None = None):
        payload = {}

        if reason:
            payload["reason"] = reason

        payload_json = json.dumps(payload)
        return self.publish(topic="robot/cmd/estop",
                            payload=payload_json,
                            qos=1,)

    def publish_arm(self,action: str)->None:
        allowed_actions = {"PICK","DROP","STOW"}

        if action not in allowed_actions:
            raise ValueError("Action must be PICK , DROP AND STOW")

        payload={"action":action}
        payload_json = json.dumps(payload)

        self.publish(
            topic="robot/cmd/arm",
            payload=payload_json,
            qos=1,
        )
    def publish_ping(self,ping_id: int)->None:
        if isinstance(ping_id, bool) or not isinstance(ping_id, int) or ping_id < 0:
            raise ValueError("ping_id must be a non-negative integer")
        payload ={
            "ping_id":ping_id,
        }
        payload_json=json.dumps(payload)

        self.publish(
            topic="robot/cmd/ping",
            payload=payload_json,
            qos=0,
        )


    def publish_console_heartbeat(
        self,
        envelope: dict[str, object],
    ) -> None:
        payload_json = json.dumps(envelope)

        self.publish(
            topic="robot/hb/console",
            payload=payload_json,
            qos=0,
        )


    def start_console_heartbeat(
        self,
        envelope_factory: Callable[[], dict[str, object]],
        rate_hz: float = 10.0,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be greater than zero")

        if (
            self._heartbeat_thread is not None
            and self._heartbeat_thread.is_alive()
        ):
            return

        self._heartbeat_stop.clear()
        interval = 1.0 / rate_hz

        def heartbeat_loop() -> None:
            next_send = time.monotonic()
            while not self._heartbeat_stop.is_set():
                remaining = next_send - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                if self._heartbeat_stop.is_set():
                    break
                try:
                    envelope = envelope_factory()
                    self.publish_console_heartbeat(envelope)
                except Exception as error:
                    print(f"Heartbeat error: {error}")


                next_send += interval
                
                now = time.monotonic()
                if next_send < now:
                    next_send = now + interval
        self._heartbeat_thread = Thread(
            target=heartbeat_loop,
            daemon=True,
        )
        self._heartbeat_thread.start()


    def stop_console_heartbeat(self) -> None:
        self._heartbeat_stop.set()

        thread = self._heartbeat_thread

        if thread is not None:
            thread.join(timeout=1.0)

            if not thread.is_alive():
                self._heartbeat_thread = None


    def publish_estop_reset(self):
        payload_json = json.dumps({})

        try:
            result = self.publish(
                topic="robot/cmd/estop_reset",
                payload=payload_json,
                qos=1,
            )
        except Exception as error:
            print(f"[MQTT] ESTOP reset publish failed: {error}")
            raise

        if result is None:
            print("[MQTT] ESTOP reset publish invoked; no broker result was returned")
        elif result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"[MQTT] ESTOP reset publish queued (mid={result.mid}); awaiting MANUAL confirmation")
        else:
            print(f"[MQTT] ESTOP reset publish rejected (rc={result.rc})")
        return result

    def subscribe_telemetry(self) -> None:
        self.subscribe(topic="robot/telemetry",
                       qos=0,)

    def subscribe_state_mode(self) -> None:
        self.subscribe(topic="robot/state/mode",
                       qos=1,)

    def subscribe_status_online(self) -> None:
        self.subscribe(topic="robot/status/online",
                       qos=1,)

    def subscribe_robot_heartbeat(self) -> None:
        self.subscribe(
            topic="robot/hb/robot",
            qos=0,
        )

    def subscribe_pong(self) -> None:
        self.subscribe(
            topic="robot/state/pong",
            qos=0,
        )

    def subscribe_log(self) -> None:
        self.subscribe(
            topic="robot/log",
            qos=0,
        )
