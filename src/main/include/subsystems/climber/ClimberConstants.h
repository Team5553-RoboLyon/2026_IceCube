#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace ClimberConstants
{
    constexpr ControlMode MainControlMode = ControlMode::MANUAL_VOLTAGE;
    constexpr ControlMode EmergencyControlMode = ControlMode::DISABLED;

    namespace Motor
    {
        constexpr int ID = 13;
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
        constexpr double climberMotor_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * Motor::KV; //RPM
        constexpr double WHEEL_RADIUS = 4.0; //in mm
        constexpr double NUMBER_OF_TEETH = 42.0;

    }

    namespace climberEncoder
    {
        constexpr int ID_CHANNEL_A = 8;
        constexpr int ID_CHANNEL_B = 9;
        constexpr bool INVERTED = false;
        
        constexpr double DISTANCE_PER_PULSE = Specifications::WHEEL_RADIUS*Specifications::NUMBER_OF_TEETH*NF64_PI/ENCODER_TICKS_PER_REVOLUTION_K2X; //in mm
    }
    
    constexpr double MAX_POSITION = 2.0; //in mm
        constexpr double MIN_POSITION = 0.0; //in mm

    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 10.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.2; //TUNEME
            constexpr double KG = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.001;
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 8.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.1; //TUNEME
            constexpr double KG = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.001;
        }
    }


    namespace Speed 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
    }
    
    namespace Settings
    {
    }
}