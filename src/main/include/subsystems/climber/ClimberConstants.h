#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if ROBOT_MODEL != COMPETITON
    #define CLIMBER_SMARTDASHBOARD_LOG
#endif
namespace ClimberConstants
{
    constexpr ControlMode MainControlMode = ControlMode::DISABLED;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_POSITION;

    namespace Motor
    {
        constexpr int ID = 1;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = 125.0; //ul
        constexpr double FREE_SPEED = Motor::VOLTAGE_COMPENSATION * Motor::KV; //RPM
        constexpr double GEAR_RADIUS = 0.021; //in m
    }

    namespace Simulation
    {
        constexpr double CARRIAGE_MASSE = 65.0; //kg 
        constexpr double MOI = 0.002; //kg.m^2 //TUNEME
        constexpr double MIN_HEIGHT = 0.000; //m //TUNEME
        constexpr double MAX_HEIGHT = 0.300; //m //TUNEME
    }

    namespace Encoder
    {
        constexpr int ID_CHANNEL_A = 5;
        constexpr int ID_CHANNEL_B = 6;
        constexpr bool INVERTED = true;
        
        constexpr double DISTANCE_PER_PULSE = Specifications::GEAR_RADIUS*2.0*NF64_PI/ENCODER_TICKS_PER_REVOLUTION_K2X; //in mm
    }
    namespace LimitSwitch
    {
        constexpr int BOTTOM_CHANNEL = 0;
        constexpr bool IS_TRIGGERED = true;

    }
    namespace IRbreaker
    {
        constexpr int CHANNEL = 7;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Gains
    {
        #if ROBOT_MODEL == SIMULATION
        namespace POSITION_VOLTAGE_PID
        {
            constexpr double KP = 1000.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 1000.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
        #else
        namespace POSITION_VOLTAGE_PID
        {
            constexpr double KP = 450.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 300.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
        #endif
    }

    namespace Setpoint
    {
        constexpr double HOME = 0.545; //TUNEME
        constexpr double ARMED = 0.74; //TUNEME
        constexpr double CLIMBED_LOCKED = 0.6; //TUNEME
        constexpr double TOLERANCE = 0.001; //TUNEME
    }

    namespace Speed 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
        constexpr double CALIBRATION = -2.0; //TUNEME
    }
    
    namespace Settings
    { 
        constexpr double BOTTOM_LIMIT = 0.545; // in m
        constexpr double TOP_LIMIT = 0.755; // in m
        constexpr double SAFETY_POSITION = 0.553; //in m
        constexpr double IRBREAKER_TRIGGER_HEIGHT = 0.61; // in m
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (TOP_LIMIT - BOTTOM_LIMIT) / (2.5/TIME_PER_CYCLE);
    }
}