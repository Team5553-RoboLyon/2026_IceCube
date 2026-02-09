#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"  
#include "units/length.h" 

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace IntakeConstants
{
    constexpr ControlMode MainControlMode = ControlMode::MANUAL_VOLTAGE;
    constexpr ControlMode EmergencyControlMode = ControlMode::DISABLED;

        namespace intakeMotor
    {
        constexpr int ID = 3;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }
    namespace pivotMotor
    {
        constexpr int ID = 4;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }

    namespace Encoder 
    {
       constexpr int ID_CHANNEL_A = 4; //TODO
       constexpr int ID_CHANNEL_B = 5; // TODO
       constexpr double REDUCTION = 1.0;
       constexpr bool INVERTED = false;
       constexpr double DISTANCE_PER_PULSE = (360.0/ENCODER_TICKS_PER_REVOLUTION_K2X)/REDUCTION; //in deg
    }

    
    
    namespace Specifications
    {
        constexpr double INTAKE_GEAR_RATIO = 3; //ul
        constexpr double PIVOT_GEAR_RATIO = 16*2.333; //ul
        constexpr double PIVOT_MIN_EXTENSION = -7; //in deg
        constexpr double PIVOT_MAX_EXTENSION = 90; //in deg
        constexpr double STARTING_POS = PIVOT_MAX_EXTENSION;
        constexpr double intakeMotor_FREE_SPEED = intakeMotor::VOLTAGE_COMPENSATION * intakeMotor::KV; //RPM
        constexpr double pivotMotor_FREE_SPEED = pivotMotor::VOLTAGE_COMPENSATION * pivotMotor::KV; //RPM
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