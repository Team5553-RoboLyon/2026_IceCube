#pragma once
#include "Lyonlib/utils/MacroUtilsRBL.h"

#define COMPETITON 0
#define TRAINING 1
#define PROTOTYPE 2
#define DEMO 3
#define SIMULATION 4

#define ADAM 0
#define VICTOR 1
#define ALEXIS 2
#define LENA 3
#define TEST 4

#define ROBOT_MODEL (TRAINING) // Change this to the desired robot model
#define PILOT (ALEXIS)
#define OPERATOR (VICTOR)

#if (ROBOT_MODEL != (COMPETITON))
#define DEBUG_MODE
#endif

constexpr double ENCODER_TICKS_PER_REVOLUTION_K2X = 2048.0;
constexpr double TIME_PER_CYCLE = 0.02; // 20ms


constexpr int OPERATOR_GAMEPAD_PORT = 2;
constexpr double OPERATOR_GAMEPAD_THRESHOLD = 0.1;