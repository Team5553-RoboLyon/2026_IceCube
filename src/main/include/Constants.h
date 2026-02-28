#pragma once
#include "Lyonlib/utils/MacroUtilsRBL.h"

#define COMPETITON 0
#define TRAINING 1
#define PROTOTYPE 2
#define DEMO 3
#define SIMULATION 4
#define BRICE 5
#define T_NOR 6

#define ADAM 0
#define VICTOR 1
#define ALEXIS 2
#define LENA 3
#define TEST 4

#define ROBOT_MODEL (PROTOTYPE) // Change this to the desired robot model
#define PILOT (ALEXIS)
#define OPERATOR (ADAM)

#if (ROBOT_MODEL != (COMPETITON))
#define DEBUG_MODE
#endif

#ifndef ROBOT_MODEL
    #error ROBOT_MODEL must be defined
#endif

#ifndef OPERATOR
    #error OPERATOR must be defined
#endif

#ifndef PILOT
    #error PILOT must be defined
#endif

inline constexpr const char* RobotToString()
{
    #if ROBOT_MODEL == COMPETITION
        return "Competition";
    #elif ROBOT_MODEL == TRAINING
        return "Training";
    #elif ROBOT_MODEL == PROTOTYPE
        return "Prototype";
    #elif ROBOT_MODEL == DEMO
        return "Demo";
    #elif ROBOT_MODEL == SIMULATION
        return "Simulation";
    #elif ROBOT_MODEL == BRICE
        return "Brice";
    #else
        #error Invalid ROBOT_MODEL value
    #endif
    
}
inline constexpr const char* PilotToString()
{
    #if PILOT == ADAM
        return "Adam";
    #elif PILOT == VICTOR
        return "Victor";
    #elif PILOT == ALEXIS
        return "Alexis";
    #elif PILOT == LENA
        return "Lena";
    #elif PILOT == TEST
        return "Test";
    #else
        return "Unknown";
    #endif
}
inline constexpr const char* OperatorToString()
{
    #if OPERATOR == ADAM
        return "Adam";
    #elif OPERATOR == VICTOR
        return "Victor";
    #elif OPERATOR == ALEXIS
        return "Alexis";
    #elif OPERATOR == LENA
        return "Lena";
    #elif OPERATOR == TEST
        return "Test";
    #else
        return "Unknown";
    #endif
}
constexpr double ENCODER_TICKS_PER_REVOLUTION_K2X = 2048.0;
constexpr double TIME_PER_CYCLE = 0.02; // 20ms



namespace ControlPanelConstants 
{
    constexpr int JOYSTICK_FORWARD_ID = 0;
    constexpr int JOYSTICK_ROTATION_ID = 1;
    constexpr int OPERATOR_GAMEPAD_PORT = 2;
    constexpr double OPERATOR_GAMEPAD_THRESHOLD = 0.1;
}  // namespace ControlPanelConstants