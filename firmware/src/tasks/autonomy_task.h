#pragma once
#include "robot_interfaces.h"
void autonomyInit();
void autonomyTask(void* arg);
void autonomyPostQr(const robot::QrResult& qr);
void autonomyPostMission(const robot::MissionConfig& mission);
