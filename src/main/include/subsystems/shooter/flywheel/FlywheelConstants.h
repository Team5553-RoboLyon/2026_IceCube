#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;


#if ROBOT_MODEL != COMPETITON
    #define FLYWHEEL_SMARTDASHBOARD_LOG
#endif
namespace FlywheelConstants
{
    constexpr ControlMode MainControlMode = ControlMode::VELOCITY_MODEL_CONTROLLED;
    constexpr ControlMode EmergencyControlMode = ControlMode::VOLTAGE;

    namespace LeftMotor
    {
        constexpr int ID = 4;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0; //TUNEME
        constexpr double CURRENT_LIMIT = 60.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
    }
    namespace RightMotor
    {
        constexpr int ID = 5;
        constexpr bool INVERTED = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kCoast;
        constexpr double VOLTAGE_COMPENSATION = 12.0; //TUNEME
        constexpr double CURRENT_LIMIT = 60.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 568.8; // RPM.V-1
    }
    
    namespace Specifications
    {
        constexpr double WHEEL_RADIUS = 2 * 0.0254; //m, 2 in
        constexpr double GEAR_RATIO = 1.0; //ul
        constexpr double LEFT_MOTOR_FREE_SPEED = LeftMotor::VOLTAGE_COMPENSATION * LeftMotor::KV; //RPM
        constexpr double RIGHT_MOTOR_FREE_SPEED = RightMotor::VOLTAGE_COMPENSATION * RightMotor::KV; //RPM
    }

    namespace Simulation
    {
        constexpr double MASS = 3.0; //kg, TUNEME
        constexpr double MOMENT_OF_INERTIA = MASS * Specifications::WHEEL_RADIUS * Specifications::WHEEL_RADIUS / 2.0; //kg.m^2, TUNEME
    }

    namespace Gains
    {
        namespace VELOCITY_VOLTAGE_PID
        {
            constexpr double KP = 0.0018; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.00003; //TUNEME
        }
        namespace FLYWHEEL_FEEDFORWARD
        {
            constexpr double KV = 1/503.0;
            constexpr double KS = 1/28.4;
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
        constexpr units::volt_t MAX = units::volt_t{RightMotor::VOLTAGE_COMPENSATION};
        constexpr units::volt_t MIN = units::volt_t{-RightMotor::VOLTAGE_COMPENSATION};
        constexpr units::volt_t REST = units::volt_t{0.0};
    }

    namespace Speed //in RPM
    {
        constexpr double REST = 0.0;
        constexpr double MAX = Specifications::LEFT_MOTOR_FREE_SPEED; //TUNEME
        constexpr double MIN = -Specifications::LEFT_MOTOR_FREE_SPEED; //TUNEME
        constexpr double TOLERANCE = 200.0; //TUNEME
        constexpr double FEED = 1200.0; //TUNEME
        constexpr double BACKWARD = -1000.0; //TUNEME
        constexpr double AGAINST_ALLIANCE_ZONE = 1000.0;
        constexpr double AGAINST_HUB = 2300.0; //TUNEME
        constexpr double TO_ALLIANCE_ZONE = 3000.0; //TUNEME
        constexpr double MAX_FOR_SHOOT = 4500.0; //TUNEME
    }
    
    namespace Settings
    {
    }
}