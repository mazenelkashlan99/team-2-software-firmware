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
constexpr const char* CMD_PING        = "robot/cmd_ping";
constexpr const char* HB_CONSOLE      = "robot/heartbeat/console";
constexpr const char* CV_QR           = "robot/cv/qr";
constexpr const char* CV_COLOR        = "robot/cv/color";

constexpr const char* TELEM           = "robot/telem";
constexpr const char* MODE_STATE      = "robot/mode_state";
constexpr const char* HB_ROBOT        = "robot/heartbeat/robot";
constexpr const char* STATUS_ONLINE   = "robot/status";
constexpr const char* PONG            = "robot/pong";
constexpr const char* EVENT           = "robot/event";
constexpr const char* DIAG            = "robot/diagnostics";

constexpr const char* PAYLOAD_ONLINE  = "online";
constexpr const char* PAYLOAD_OFFLINE = "offline";

} // namespace topic
