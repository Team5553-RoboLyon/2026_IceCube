#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;


#if ROBOT_MODEL != COMPETITON
    #define HOOD_SMARTDASHBOARD_LOG
#endif

namespace HoodConstants
{
    constexpr ControlMode MainControlMode = ControlMode::DISABLED; //MANUAL_POSITION
    constexpr ControlMode EmergencyControlMode = ControlMode::POSITION_VOLTAGE_PID;
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
        constexpr int KV = 985.6; // RPM.V-1
    }

    namespace HoodEncoder
    {
        constexpr int ID_CHANNEL_A = 19;
        constexpr int ID_CHANNEL_B = 20;
        constexpr bool INVERTED = true;
        constexpr double RATIO = 215.0/14.0; 
        constexpr double DISTANCE_PER_PULSE = 2.0*NF64_PI/RATIO/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = HoodEncoder::RATIO * (5.23) * (5.23) * (5.23); //ul 
        constexpr double HOOD_MOTOR_FREE_SPEED = HoodMotor::VOLTAGE_COMPENSATION * HoodMotor::KV; //RPM
    }

    namespace Simulation
    {
        constexpr double CARRIAGE_MASSE = 65.0; //kg 
        constexpr double MOI = 1.0; //kg.m^2 //TUNEME
        constexpr double ARM_LENGTH = 0.33; //m //TUNEME
    }

    namespace Gains
    {
        namespace POSITION_VOLTAGE_PID
        {
            constexpr double KP = 125.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
        namespace MANUAL_POSITION
        {
            constexpr double KP = 125.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
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
        constexpr units::volt_t MAX = units::volt_t{HoodMotor::VOLTAGE_COMPENSATION}; 
        constexpr units::volt_t MIN = units::volt_t{-HoodMotor::VOLTAGE_COMPENSATION};
        constexpr units::volt_t REST = units::volt_t{0.0};
    }

    namespace Position //in rad
    { 
        constexpr double MAX = NDEGtoRAD(19.1); //TUNEME
        constexpr double MIN = 0.0; //TUNEME
        constexpr double TOLERANCE = NDEGtoRAD(0.1); //TUNEME
        constexpr double FEED = MAX;  //TUNEME
    }
    
    namespace Settings
    {
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (Position::MAX - Position::MIN) / (0.02/TIME_PER_CYCLE);
    }
}