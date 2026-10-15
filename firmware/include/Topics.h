#pragma once

// Single source of truth for every MQTT topic string.
namespace topic {

constexpr const char* CMD_DRIVE       = "robot/cmd/drive";
constexpr const char* CMD_MODE        = "robot/cmd/mode";
constexpr const char* CMD_ESTOP       = "robot/cmd/estop";
constexpr const char* CMD_ESTOP_RESET = "robot/cmd/estop_reset";
constexpr const char* CMD_START       = "robot/cmd/start";
constexpr const char* CMD_MISSION     = "robot/cmd/mission";
constexpr const char* CMD_GOTO        = "robot/cmd/goto";
constexpr const char* CMD_ARM         = "robot/cmd/arm";
constexpr const char* CMD_POSE_SET    = "robot/cmd/pose_set";
constexpr const char* CMD_PING        = "robot/cmd/ping";
constexpr const char* HB_CONSOLE      = "robot/heartbeat/console";
constexpr const char* CV_QR           = "robot/cv/qr";
constexpr const char* CV_COLOR        = "robot/cv/color";

constexpr const char* TELEM           = "robot/state/telemetry";
constexpr const char* MODE_STATE      = "robot/state/mode";
constexpr const char* HB_ROBOT        = "robot/hb/robot";
constexpr const char* STATUS_ONLINE   = "robot/status/online";
constexpr const char* PONG            = "robot/state/pong";
constexpr const char* EVENT           = "robot/status/event";
constexpr const char* LOG             = "robot/log";

constexpr const char* PAYLOAD_ONLINE  = "online";
constexpr const char* PAYLOAD_OFFLINE = "offline";

} // namespace topic
