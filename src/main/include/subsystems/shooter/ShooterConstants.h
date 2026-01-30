#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace ShooterConstants
{
    constexpr ControlMode MainControlMode = ControlMode::VOLTAGE;
    constexpr ControlMode EmergencyControlMode = ControlMode::DISABLED;

    namespace LeftMotor
    {
        constexpr int ID = 1;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
    }
    namespace RightMotor
    {
        constexpr int ID = 2;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
    }
    namespace BottomMotor
    {
        constexpr int ID = 3;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
    }

    namespace WheelEncoder 
    {
        constexpr int A_ID = 1;
        constexpr int B_ID = 2;
        constexpr bool REVERSED = false;
        constexpr double DISTANCE_PER_PULSE = (1) / ENCODER_TICKS_PER_REVOLUTION_K2X;
    }
    namespace IRBreakerOutput 
    {
        constexpr int ID = 3;
        constexpr bool IS_TRIGGERED = false;
    }
    
    namespace Specifications
    {
        constexpr double GEAR_RATIO = 1; //ul
        constexpr double LeftMotor_FREE_SPEED = LeftMotor::VOLTAGE_COMPENSATION * LeftMotor::KV; //RPM
        constexpr double RightMotor_FREE_SPEED = RightMotor::VOLTAGE_COMPENSATION * RightMotor::KV; //RPM
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