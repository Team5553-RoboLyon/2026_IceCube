#pragma once

#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"  
#include "units/length.h" 

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace PivotConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_VOLTAGE;

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
       constexpr double FULL_RANGE = 2.0*M_PI;
       constexpr double EXPECTED_ZERO = 0.0;//TUNEME
    }

    namespace EncoderRight
    {
       constexpr int ID = 0;
       constexpr bool INVERTED = false;
       constexpr double FULL_RANGE = 2.0*M_PI;
       constexpr double EXPECTED_ZERO = 0.0;//TUNEME
    }

    namespace Specifications
    {
        constexpr double PIVOT_GEAR_RATIO = 16*2.333; //ul
        constexpr double PIVOT_HOME_POS = NDEGtoRAD(-7.0); //in rad
        constexpr double PIVOT_EXTENDED_POS = M_PI; //in rad
        constexpr double POS_TOLERANCE = 1.0; //in deg
        constexpr double pivotMotor_FREE_SPEED = pivotMotor::VOLTAGE_COMPENSATION * pivotMotor::KV; //RPM
    } 

    namespace Gains
    {
        namespace PIVOT_POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 0.001; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double KG = 0.0; //TUNEME
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