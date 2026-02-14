#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace HoodConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_VOLTAGE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::VOLTAGE;

    namespace Specifications
    {
        constexpr double GEAR_RATIO = 1; //ul TUNEME
    }

    namespace HoodMotor
    {
        constexpr int ID = 3;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
        constexpr double ENCODER_DISTANCE_PER_PULSE = 2*M_PI/Specifications::GEAR_RATIO/42.0;
    }

    namespace Specifications
    {
        constexpr double HoodMotor_FREE_SPEED = HoodMotor::VOLTAGE_COMPENSATION * HoodMotor::KV; //RPM
    }

    namespace Gains
    {
        namespace POSITION_VOLTAGE_PID
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
        constexpr double MAX = HoodMotor::VOLTAGE_COMPENSATION; 
        constexpr double MIN = -HoodMotor::VOLTAGE_COMPENSATION;
        constexpr double REST = 0.0;
    }

    namespace Position //in rad
    { 
        constexpr double MAX = NDEGtoRAD(17.0); //TUNEME
        constexpr double MIN = 0.0; //TUNEME
        constexpr double TOLERANCE = NDEGtoRAD(0.1); //TUNEME
        constexpr double FEED = MAX;  //TUNEME
        constexpr double TO_ALLIANCE_ZONE = NDEGtoRAD(8.5); //TUNEME
    }
    
    namespace Settings
    {
    }
}