import paho.mqtt.client as mqtt
from typing import Callable
from paho.mqtt.client import CallbackAPIVersion
import json
from threading import Event, Thread
import time

class MQTTClient:
    def __init__(
            self,
            broker_host: str = "localhost",      #Address of broker
            broker_port: int = 1883,
            client_id: str = "rov_software",
            heartbeat_envelope_factory: Callable[[], dict[str, object]] | None = None,
                ):
        self.broker_host = broker_host
        self.broker_port = broker_port

        self._heartbeat_stop = Event()
        self._heartbeat_thread: Thread | None = None
        self._heartbeat_seq = 0
        self._heartbeat_envelope_factory=( 
            heartbeat_envelope_factory
            if heartbeat_envelope_factory is not None
            else self._create_heartbeat_envelope)
        self.client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2,
                                  client_id=client_id,)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self.message_callback: Callable | None = None


        



    def _create_heartbeat_envelope(self)-> dict [str,object]:
        self._heartbeat_seq +=1
        return{ "v":1,
               "seq": self._heartbeat_seq,
               "ts": int(time.time()*1000),}
    def connect(self) -> None:                      #handeling the broker connection
        self.client.connect(
            self.broker_host,
            self.broker_port,
        )
        self.client.loop_start()                   #turning network loop in the background thread

    def disconnect(self) -> None:
        self.stop_console_heartbeat()
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
        self.subscribe_telemetry()
        self.subscribe_state_mode()
        self.subscribe_status_online()
        self.subscribe_status_event()
        self.subscribe_robot_heartbeat()
        self.subscribe_pong()
        self.subscribe_log()
        if self._heartbeat_envelope_factory is not None:
            self.start_console_heartbeat(
                self._heartbeat_envelope_factory,rate_hz=10.0,
            )
        print("All subscriptions requested.")

    def _on_disconnect(self,client,userdata,flags,reason_code,properties,) -> None:
        print(f"DisConnected to MQTT broker: {reason_code}")
        self.stop_console_heartbeat()


    def _on_message(self,client,userdata,message,)->None:
        payload = message.payload.decode("utf-8")
        if self.message_callback:
            self.message_callback(
                message.topic,
                payload,
            )

    def publish_drive(self,thr:float,turn:float) -> None:
        if not -1 <= thr <=1:
            raise ValueError("thr must be between -1 and 1")
        if not -1 <= turn <=1:
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

    def publish_mode(self,mode:str) ->None:
        allowed_modes = {"MANUAL","SEMI","AUTO"}

        if mode  not in allowed_modes:
            raise ValueError("Mode must be MANUAL,SEMI OR AUTO")

        payload={"mode" : mode,}
        payload_json = json.dumps(payload)
        self.publish(topic="robot/cmd/mode",
                     payload=payload_json,
                     qos=1,)

    def publish_estop(self,reason:str | None = None) -> None:
        payload = {}

        if reason:
            payload["reason"] = reason

        payload_json = json.dumps(payload)
        self.publish(topic="robot/cmd/estop",
                     payload=payload_json,
                     qos=1,)

    def publish_start(self)->None:
        payload = {}

        payload_json = json.dumps(payload)

        self.publish(
            topic="robot/cmd/start",
            payload=payload_json,
            qos=1,
        )

    def publish_mission(self,mission:dict) -> None:
        if not isinstance(mission,dict):
            raise ValueError("mission must be a dictionary")
        boxes = mission.get("boxes")
        zones = mission.get("zones")
        if not isinstance(boxes,list):
            raise ValueError("mission must contain a a boxes list")

        if not isinstance(zones,list):
            raise ValueError("mission must contain a a zones list")

        if len(boxes) > 8 :
            raise ValueError("A mission cannot contain more than 8 boxes")

        if len(zones) > 4 :
            raise ValueError("A mission cannot contain more than 4 zones")
        for index,box in enumerate(boxes):
            if not isinstance(box,dict):
                raise ValueError(f"boxes[{index}] must be a dictionary")
            required_fields = {"id", "type", "x", "y"}
            missing_fields = required_fields - box.keys()
            if missing_fields:
                raise ValueError(
                    f"boxes[{index}] missing fields: "
                    f"{', '.join(sorted(missing_fields))}"
                )
            if (
                isinstance(box["id"],bool) 
                or not isinstance(box["id"],(int,str))
                or box["id"] == ""
            ):
                raise ValueError (f"boxes[{index}].id must be an integer or non-empty string")
            if not isinstance(box["type"],str) or not box ["type"]:
                raise ValueError(
                    f"boxes[{index}].type must be a non-empty string"
                )
            for field in ("x", "y"):
                value = box[field]

                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(
                        f"boxes[{index}].{field} must be numeric"
                    )

        for index, zone in enumerate(zones):
            if not isinstance(zone, dict):
                raise ValueError(f"zones[{index}] must be a dictionary")

            required_fields = {"type", "x", "y"}
            missing_fields = required_fields - zone.keys()

            if missing_fields:
                raise ValueError(
                    f"zones[{index}] missing fields: "
                    f"{', '.join(sorted(missing_fields))}"
                )

            if not isinstance(zone["type"], str) or not zone["type"]:
                raise ValueError(
                    f"zones[{index}].type must be a non-empty string"
                )

            for field in ("x", "y"):
                value = zone[field]

                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(
                        f"zones[{index}].{field} must be numeric"
                    )

        payload_json = json.dumps(mission)

        self.publish(
            "robot/cmd/mission",
            payload=payload_json,
            qos=1,
        )

    def publish_goto(self,target : dict[str,object])->None:
        payload_json = json.dumps(target)

        self.publish(
            topic="robot/cmd/goto",
            payload=payload_json,
            qos=1,
        )

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
    def publish_pose_set(self,x:float,y:float,theta:float,) ->None:
        payload ={"x" :x,
                  "y":y,
                  "theta":theta,}
        payload_json=json.dumps(payload)

        self.publish(topic="robot/cmd/pose_set",
                     payload=payload_json,
                     qos=1,
                     )

    def publish_ping(self,ping_id: int,t_send:float)->None:
        payload ={
            "ping_id":ping_id,
            "t_send":t_send,
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
            while not self._heartbeat_stop.is_set():
                try:
                    envelope = envelope_factory()
                    self.publish_console_heartbeat(envelope)
                except Exception as error:
                    print(f"Heartbeat error: {error}")

                self._heartbeat_stop.wait(interval)

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


    def publish_qr(self,valid:bool,raw:str,x:float | None , y:float|None,) -> None:
        payload = {"valid":valid,
                   "raw":raw,
                   "x":x,
                   "y":y,
                   }
        payload_json = json.dumps(payload)
        self.publish(topic="robot/cv/qr",
                     payload=payload_json,
                     qos=1,
                     )

    def publish_color(self,color_data: dict[str,object])-> None:
        payload_json = json.dumps(color_data)
        self.publish(
            topic="robot/cv/color",
            payload=payload_json,
            qos=1,
        )


    def publish_estop_reset(self) -> None:
        payload_json = json.dumps({})

        self.publish(
            topic="robot/cmd/estop_reset",
            payload=payload_json,
            qos=1,
        )

    def subscribe_telemetry(self) -> None:
        self.subscribe(topic="robot/state/telemetry",
                       qos=0,)

    def subscribe_state_mode(self) -> None:
        self.subscribe(topic="robot/state/mode",
                       qos=1,)

    def subscribe_status_online(self) -> None:
        self.subscribe(topic="robot/status/online",
                       qos=1,)

    def subscribe_status_event(self) -> None:
        self.subscribe(
            topic="robot/status/event",
            qos=1,
        )

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
