#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace FlywheelConstants
{
    constexpr ControlMode MainControlMode = ControlMode::VELOCITY_VOLTAGE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::VOLTAGE;

    namespace LeftMotor
    {
        constexpr int ID = 5;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0; //TUNEME
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
    }
    namespace RightMotor
    {
        constexpr int ID = 4;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0; //TUNEME
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
    }
    
    namespace Specifications
    {
        constexpr double GEAR_RATIO = 1; //ul
        constexpr double LeftMotor_FREE_SPEED = LeftMotor::VOLTAGE_COMPENSATION * LeftMotor::KV; //RPM
        constexpr double RightMotor_FREE_SPEED = RightMotor::VOLTAGE_COMPENSATION * RightMotor::KV; //RPM
    }

    namespace Gains
    {
        namespace VELOCITY_VOLTAGE_PID
        {
            constexpr double KP = 10.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.2; //TUNEME
        }
    }

    namespace DutyCycle
    {
        constexpr double MAX = 1.0;
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
    }


    namespace Voltage 
    {
        constexpr double MAX = RightMotor::VOLTAGE_COMPENSATION;
        constexpr double MIN = -RightMotor::VOLTAGE_COMPENSATION;
        constexpr double REST = 0.0;
    }

    namespace Speed //in RPM
    {
        constexpr double REST = 0.0;
        constexpr double MAX = Specifications::LeftMotor_FREE_SPEED; //TUNEME
        constexpr double MIN = -Specifications::LeftMotor_FREE_SPEED; //TUNEME
        constexpr double TOLERANCE = 0.1; //TUNEME
        constexpr double FEED = 1200.0; //TUNEME
        constexpr double BACKWARD = -1000.0; //TUNEME
        constexpr double TO_ALLIANCE_ZONE = 5000.0; //TUNEME
        constexpr double AGAINST_HUB = 2000.0; //TUNEME
        constexpr double MAX_FOR_SHOOT = 4500.0; //TUNEME
    }
    
    namespace Settings
    {
    }
}