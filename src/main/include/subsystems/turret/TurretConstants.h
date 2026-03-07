#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;


#if ROBOT_MODEL != COMPETITION
    #define TURRET_SMARTDASHBOARD_LOG
#endif
namespace TurretConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_POSITION;

    namespace Motor
    {
        constexpr int ID = 2;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.4;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 985.6; // RPM.V-1
    }
    
    namespace Specifications
    {
        constexpr double MOTOR_RATIO = (5.23)*(5.23);
        constexpr double GEAR_RATIO = (130.0/24.0)*MOTOR_RATIO; //ul
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * Motor::KV; //RPM
        constexpr frc::Transform2d ROBOT_TO_TURRET{155.0_m, 155.0_m,{}}; //TUNEME
    }

    namespace Encoder 
    {
        constexpr int A_ID = 22;
        constexpr int B_ID = 23;
        constexpr bool REVERSED = false;
        constexpr double DISTANCE_PER_PULSE = 2*NF64_PI/(Specifications::GEAR_RATIO/Specifications::MOTOR_RATIO)/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace HallEffectSensor
    {
        constexpr int ID = 2;
        constexpr double MIN_VALUE_WHEN_MAGNET = 790; //TUNEME
    }

    namespace Simulation
    {
        constexpr double MASS = 6.5; //kg, TUNEME
        constexpr double RADIUS = 0.3; //m, TUNEME
        constexpr double MOI = 0.5 * MASS * RADIUS * RADIUS; //kg*m^2, TUNEME
    }
    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 2.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
        }
    }

    namespace Setpoints
    {
        constexpr double EJECT = NF64_PI; //TUNEME
        constexpr double TOLERANCE = NDEGtoRAD(2.0);
        constexpr double INIT_POS = NF64_PI_2;
    }

    namespace DutyCycle 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
        constexpr double INIT = 0.1;
    }
    
    namespace Settings
    {
        constexpr double BOTTOM_LIMIT = NDEGtoRAD(-132.0); //TUNEME
        constexpr double TOP_LIMIT = NDEGtoRAD(138.0); //TUNEME
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (TOP_LIMIT - BOTTOM_LIMIT) / (2.0/TIME_PER_CYCLE); //TUNEME
        constexpr double OPEN_LOOP_REDUC = 10.0;
    }
}