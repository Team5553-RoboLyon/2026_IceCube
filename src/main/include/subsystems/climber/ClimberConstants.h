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

        namespace climberMotor
    {
        constexpr int ID = 13;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }

    namespace climberEncoder
    {
        constexpr int ID_CHANNEL_A = 8;
        constexpr int ID_CHANNEL_B = 9;
        constexpr bool INVERTED = false;
        
        constexpr double DISTANCE_PER_PULSE = 2.0*42.0*NF64_PI/2048.0; //in mm
    }

    
    
    namespace Specifications
    {
        constexpr double GEAR_RATIO = 125.0; //ul
        constexpr double climberMotor_FREE_SPEED = climberMotor::VOLTAGE_COMPENSATION * climberMotor::KV; //RPM
        constexpr double MAX_POSITION = 2.0; //in mm
        constexpr double MIN_POSITION = 0.0; //in mm
    }

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