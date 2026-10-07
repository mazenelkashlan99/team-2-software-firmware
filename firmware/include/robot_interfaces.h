// robot_interfaces.h  --  shared firmware contract (Persons A, B, C)
// Version: v0 DRAFT
//
// Rules
//  - Units everywhere inside the firmware: meters, radians, seconds.
//    Conversion to other units happens only at the console boundary.
//  - Plain C++ only: no Arduino.h, no FreeRTOS headers. This file must
//    compile in env:native so that pure-logic libraries can be unit-tested.
//  - [SPEC] = taken from the Firmware Work Details PDF / rulebook.
//  - [PROPOSAL] = not specified by the PDF; Person C's recommendation,
//    to be confirmed by the team before the header is frozen.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace robot
{

    // ---------------------------------------------------------------------------
    // Time base [PROPOSAL]
    // Timestamps are milliseconds

    // ---------------------------------------------------------------------------
    typedef uint32_t TimeMs;

    // ---------------------------------------------------------------------------
    // Mode [SPEC: MANUAL, SEMI, AUTO, ESTOP]
    // ---------------------------------------------------------------------------
    enum class Mode : uint8_t
    {
        MANUAL = 0,
        SEMI = 1,
        AUTO = 2,
        ESTOP = 3
    };

    // ---------------------------------------------------------------------------
    // Command [SPEC: {v, w}]
    // Producer: Person C command arbiter (autonomy_task).
    // Consumer: Person A control_task / DriveBase.
    // Delivery: length-1 overwrite slot, refreshed >= 20 Hz [PROPOSAL].
    // ---------------------------------------------------------------------------
    struct Command
    {
        float v = 0.0f; // linear velocity,  m/s   (+ forward)
        float w = 0.0f; // angular velocity, rad/s (+ counter-clockwise) [PROPOSAL sign]
    };

    // ---------------------------------------------------------------------------
    // Pose [SPEC: {x, y, theta, v, w, timestamp}]
    // Producer: Person B pose_task, 100 Hz. Owner: Person B.
    // Consumers: autonomy_task, comms_task (telemetry).
    // Access:   getPose() / resetPose(x,y,theta) [SPEC names]; copy under
    //           a short mutex/critical section (never hold across a slow call).
    // Frame [PROPOSAL, must be confirmed by B]: origin = start point (0,0);
    //           +x to the right, +y toward the box row (y = 3 m);
    //           theta = 0 along +x, positive counter-clockwise, wrapped to
    //           (-pi, pi].
    // Range:    x, y in about [-0.5, 3.5] m on the 3 x 3 m field.
    // ---------------------------------------------------------------------------
    struct Pose
    {
        float x = 0.0f;     // m
        float y = 0.0f;     // m
        float theta = 0.0f; // rad
        float v = 0.0f;     // m/s, linear speed
        float w = 0.0f;     // rad/s, angular speed
        TimeMs t = 0;       // ms since boot, time of this estimate
        bool valid = false; // [PROPOSAL] false until calibration done / sensor lost
    };

    // ---------------------------------------------------------------------------
    // Arm [SPEC: ArmCmd{pick, drop, stow}, ArmStatus{idle, busy, done, failed}]
    // ArmCmd:    producer Person C (autonomy or manual passthrough),
    //            consumer Person A arm_task (notification / queue).
    // ArmStatus: producer Person A arm_task, consumer Person C. Polled.
    // Behaviour for a new ArmCmd while busy is UNSPECIFIED -> asked to A.
    // ---------------------------------------------------------------------------
    enum class ArmCmd : uint8_t
    {
        PICK = 0,
        DROP = 1,
        STOW = 2
    };

    enum class ArmStatus : uint8_t
    {
        IDLE = 0,
        BUSY = 1,
        DONE = 2,
        FAILED = 3
    };

    // ---------------------------------------------------------------------------
    // QrResult [name from file-structure image; fields are a PROPOSAL]
    // Producer: CV (laptop) via MQTT robot/cv/qr, QoS1.
    // Consumer: autonomy_task (AUTO), console (SEMI display / go-to).
    // The QR text is "X=<m>&Y=<m>", e.g. "X=1.56&Y=0.49" [SPEC: rulebook].
    // Error:    valid=false or destination outside the field -> rejected.
    // ---------------------------------------------------------------------------
    static const size_t QR_RAW_MAX = 32;

    struct QrResult
    {
        bool valid = false;
        float x = 0.0f;             // destination, m, from start point
        float y = 0.0f;             // destination, m
        uint32_t seq = 0;           // sender sequence number
        char raw[QR_RAW_MAX] = {0}; // original string, NUL-terminated
    };

    // ---------------------------------------------------------------------------
    // I added this part as we need it for the mission config when we are in AUTO mode
    // It's not specified in the PDF, so any changes are welcome to discuss
    // the mission config is sent from the console to the robot via MQTT
    // Mission data [ box and zone coordinates
    // come from the console before the round starts]
    // Producer: console via MQTT robot/cmd/mission, QoS1.
    // Consumer: autonomy_task / MissionPlanner.
    // Zone positions change between rounds, so this is sent every round.
    // ---------------------------------------------------------------------------
    static const size_t MAX_BOXES = 8; // field figure shows 8 boxes
    static const size_t MAX_ZONES = 4;

    enum class BoxType : uint8_t
    {
        BLUE_EASY = 0, // zone present the whole run
        RED_HARD = 1,  // x5 if delivered within the first 4 minutes
        QR = 2         // destination from QrResult; no zone
    };

    struct BoxTarget
    {
        uint8_t id = 0;
        BoxType type = BoxType::BLUE_EASY;
        float x = 0.0f; // pickup position, m
        float y = 0.0f; // pickup position, m
    };

    struct ZoneTarget
    {
        BoxType type = BoxType::BLUE_EASY; // which boxes drop here (never QR)
        float x = 0.0f;                    // zone center, m
        float y = 0.0f;                    // zone center, m
    };

    struct MissionConfig
    {
        BoxTarget boxes[MAX_BOXES];
        uint8_t boxCount = 0;
        ZoneTarget zones[MAX_ZONES];
        uint8_t zoneCount = 0;
    };

}