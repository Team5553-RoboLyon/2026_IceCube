#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace IntakeConstants
{
    constexpr ControlMode MainControlMode = ControlMode::MANUAL_VELOCITY;
    constexpr ControlMode EmergencyControlMode = ControlMode::DISABLED;

        namespace leftMotor
    {
        constexpr int ID = 3;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }
    namespace rightMotor
    {
        constexpr int ID = 4;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }

    
    
    namespace Specifications
    {
        constexpr double GEAR_RATIO = 3; //ul
                constexpr double leftMotor_FREE_SPEED = leftMotor::VOLTAGE_COMPENSATION * leftMotor::KV; //RPM
        constexpr double rightMotor_FREE_SPEED = rightMotor::VOLTAGE_COMPENSATION * rightMotor::KV; //RPM
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