#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace IndexerConstants
{
    constexpr ControlMode MainControlMode = ControlMode::FEEDFORWARD_VELOCITY_VOLTAGE;
    constexpr ControlMode EmergencyControlMode = ControlMode::VOLTAGE;

    namespace indexerMotor
    {
        constexpr int ID = 7;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }

    namespace clodeMotor
    {
        constexpr int ID = 6;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
        constexpr double GEAR_RATIO = 45.0; //TUNEME
    }

    namespace theMostImportantSensorOfTheRobot //IndexerIRbreaker
    {
        constexpr int ID = 18;
        constexpr bool IS_TRIGERED = true; //TUNEME
    }

    
    
    namespace Specifications
    {
        constexpr double GEAR_RATIO = 3.0; //ul
        constexpr double indexerMotor_FREE_SPEED = indexerMotor::VOLTAGE_COMPENSATION * indexerMotor::KV; //RPM
        constexpr double Clode_FREE_SPEED = clodeMotor::VOLTAGE_COMPENSATION * clodeMotor::KV; //RPM
    }

    namespace Gains
    {
        namespace FEEDFORWARD_VELOCITY_VOLTAGE
        {
            constexpr double KS = 0.0; //TUNEME
            constexpr double KV = 0.0; //TUNEME
            constexpr double KA = 0.0; //TUNEME
        }
    }

    namespace Simulation //very aproximative values
    {
        constexpr double MOI = 0.05310484;
        constexpr double CLODE_MOI = 0.3920869; 
    }


    namespace Voltage //TUNEME
    {
        constexpr double REST = 0.0;
        constexpr double MAX = indexerMotor::VOLTAGE_COMPENSATION; 
        constexpr double MIN = -indexerMotor::VOLTAGE_COMPENSATION;
        constexpr double FEED = 12.0;
        constexpr double CLODE_POWER = -10.0;
    }

    namespace Speed //in RPM
    {
        constexpr double REST = 0.0;
        constexpr double FEED = 2500.0;
        constexpr double EVACUATE = -1000.0;
        constexpr double PREPARE_SHOOT = 1000.0;
    }
    
    namespace Settings
    {
    }
}