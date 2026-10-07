// Topics.h -- MQTT topic names (v1 draft, see docs/interfaces.md)
// Broker: Eclipse Mosquitto on the laptop. QoS/retain are noted per topic.
#pragma once

namespace robot
{
    namespace topic
    {

        // ---- Console -> robot ------------------------------------------------------
        static const char *const CMD_DRIVE = "robot/cmd/drive";             // QoS0 ~30 Hz
        static const char *const HB_CONSOLE = "robot/hb/console";           // QoS0 10 Hz
        static const char *const CMD_MODE = "robot/cmd/mode";               // QoS1
        static const char *const CMD_ESTOP = "robot/cmd/estop";             // QoS1
        static const char *const CMD_ESTOP_RESET = "robot/cmd/estop_reset"; // QoS1
        static const char *const CMD_START = "robot/cmd/start";             // QoS1
        static const char *const CMD_MISSION = "robot/cmd/mission";         // QoS1
        static const char *const CMD_GOTO = "robot/cmd/goto";               // QoS1
        static const char *const CMD_ARM = "robot/cmd/arm";                 // QoS1
        static const char *const CMD_POSE_SET = "robot/cmd/pose_set";       // QoS1
        static const char *const CMD_PING = "robot/cmd/ping";               // QoS0

        // ---- CV -> robot -----------------------------------------------------------
        static const char *const CV_QR = "robot/cv/qr";       // QoS1
        static const char *const CV_COLOR = "robot/cv/color"; // QoS1

        // ---- Robot -> console ------------------------------------------------------
        static const char *const STATE_TELEMETRY = "robot/state/telemetry"; // QoS0 10-20 Hz
        static const char *const STATE_MODE = "robot/state/mode";           // QoS1 retained
        static const char *const STATE_PONG = "robot/state/pong";           // QoS0
        static const char *const STATUS_ONLINE = "robot/status/online";     // QoS1 retained, LWT
        static const char *const STATUS_EVENT = "robot/status/event";       // QoS1
        static const char *const HB_ROBOT = "robot/hb/robot";               // QoS0 5 Hz
        static const char *const LOG = "robot/log";                         // QoS0

        // Plain-text payloads of robot/status/online (not JSON)
        static const char *const PAYLOAD_ONLINE = "online";
        static const char *const PAYLOAD_OFFLINE = "offline";

        // Wildcard subscriptions used by the robot
        static const char *const SUB_CMD = "robot/cmd/#";
        static const char *const SUB_CV = "robot/cv/#";
        static const char *const SUB_HB_CONSOLE = "robot/hb/console";

    } // namespace topic
} // namespace robot