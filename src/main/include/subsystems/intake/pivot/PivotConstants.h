#pragma once

#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"  
#include "units/length.h" 
#include "units/moment_of_inertia.h"
#include "LyonLib/utils/MacroUtilsRBL.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace PivotConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_DUTY_CYCLE;

    namespace pivotMotor
    {
        constexpr int ID = 8;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }   
    
    namespace EncoderLeft
    {
       constexpr int ID = 8;
       constexpr bool INVERTED = true;
       constexpr double FULL_RANGE = 2.0*NF64_PI;
       constexpr double EXPECTED_ZERO = 0.0;//TUNEME
    }

    namespace EncoderRight
    {
       constexpr int ID = 9;
       constexpr bool INVERTED = false;
       constexpr double FULL_RANGE = 2.0*NF64_PI;
       constexpr double EXPECTED_ZERO = 0.0;//TUNEME
    }

    namespace Specifications
    {
        constexpr double PIVOT_GEAR_RATIO = 16*2.333; //ul
        constexpr double pivotMotor_FREE_SPEED = pivotMotor::VOLTAGE_COMPENSATION * pivotMotor::KV; //RPM
    } 

    namespace Simulation //approximative values
    {
        constexpr units::kilogram_square_meter_t MOI = 0.2_kg_sq_m;
        constexpr units::meter_t ARM_LENGTH = 0.31_m;
        constexpr bool APPLY_GRAVITY = true;
    }

    namespace Position //in rad TUNEME
    {
        constexpr double MIN = NDEGtoRAD(5.0);
        constexpr double MAX = NF64_PI/2.0;
        constexpr double RANGE = MAX-MIN;
        constexpr double EXTENDED_POS = MIN;
        constexpr double HOME_POS = MAX;
        constexpr double SAFETY_POS = NDEGtoRAD(61.0);
        constexpr double POS_TOLERANCE = NDEGtoRAD(1.0);
    }

    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 0.2; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double KG = 0.0; //TUNEME
        }
    }

    namespace DutyCycle 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
        constexpr double INDEXER_MODE = 0.2; //TUNEME
    }
    
    namespace Settings
    {
    }
}