#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace ClimberConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_POSITION;

    namespace Motor
    {
        constexpr int ID = 1;
        constexpr bool INVERTED = false;

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
        constexpr double WHEEL_RADIUS = 2.0; //in mm
        constexpr double NUMBER_OF_TEETH = 42.0;
    }

    namespace Encoder
    {
        constexpr int ID_CHANNEL_A = 8;
        constexpr int ID_CHANNEL_B = 9;
        constexpr bool INVERTED = false;
        
        constexpr double DISTANCE_PER_PULSE = Specifications::WHEEL_RADIUS*Specifications::NUMBER_OF_TEETH*NF64_PI/ENCODER_TICKS_PER_REVOLUTION_K2X; //in mm
    }
    namespace LimitSwitch
    {
        constexpr int BOTTOM_CHANNEL = 0;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace HallEffectSensor
    {
        constexpr int CHANNEL = 0;
    }
    namespace Gains
    {
        namespace POSITION_VOLTAGE_PID
        {
            constexpr double KP = 5.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 5.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.001; //TUNEME
        }
    }

    namespace Setpoint
    {
        constexpr double HOME = 0.050; //TUNEME
        constexpr double ARMED = 0.200; //TUNEME
        constexpr double CLIMBED_LOCKED = 0.050; //TUNEME
        constexpr double TOLERANCE = 0.001; //TUNEME
    }

    namespace Speed 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
        constexpr double CALIBRATION = -0.01; //TUNEME
    }
    
    namespace Settings
    {
        constexpr double BOTTOM_LIMIT = 0.000; // in m //TUNEME
        constexpr double TOP_LIMIT = 0.300; // in m //TUNEME 
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (TOP_LIMIT - BOTTOM_LIMIT) / (2.5/TIME_PER_CYCLE); //TUNEME
    }
}