#pragma once

#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"  
#include "units/length.h" 
#include "units/moment_of_inertia.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace RollerConstants
{
    
    constexpr ControlMode MainControlMode = ControlMode::VOLTAGE;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_VOLTAGE;

    namespace rollerMotor
    {
        constexpr int ID = 13;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }

    namespace Simulation //approximative values
    {
        constexpr units::kilogram_square_meter_t MOI = 0.36_kg_sq_m;
    }

    namespace Specifications
    {
        constexpr double ROLLER_GEAR_RATIO = 3; //ul
        constexpr double rollerMotor_FREE_SPEED = rollerMotor::VOLTAGE_COMPENSATION * rollerMotor::KV; //RPM
    } 

    namespace Gains
    {
    }

    namespace Voltage
    {
        constexpr double REST = 0.0;
        constexpr double REFUEL = 12.0; //TUNEME
        constexpr double IM_AN_INDEXER = 4.0; //TUNEME
        constexpr double EJECT = -10.0;
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