#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"   
#include "camera/AprilTagPhotonCamera.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace TurretConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_POSITION;

    namespace motor
    {
        constexpr int ID = 3;
        constexpr bool INVERTED = false;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.4;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
        constexpr int KV = 496.3; // RPM.V-1
    }
    
    namespace Specifications
    {
        constexpr double GEAR_RATIO = 130.0/24.0; //ul
        constexpr double motor_FREE_SPEED = motor::VOLTAGE_COMPENSATION * motor::KV; //RPM
        constexpr frc::Transform2d ROBOT_TO_TURRET {162.5_m, 162.5_m,{}};
        constexpr double EJECT_POS = 0.0; //TUNEME
    }

    namespace encoder 
    {
        constexpr int A_ID = 23;
        constexpr int B_ID = 24;
        constexpr bool REVERSED = false;
        constexpr double DISTANCE_PER_PULSE = 2*M_PI/Specifications::GEAR_RATIO/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace TurretCamera
    {
        const std::string NAME = "Big brother"; //TUNEME
    }

    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 0.1; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double TOLERANCE = NDEGtoRAD(0.1);
        }
    }


    namespace DutyCycle 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double REST = 0.0;
    }
    
     namespace Settings
    {
        constexpr double BOTTOM_LIMIT = -2*M_PI; //TUNEME
        constexpr double TOP_LIMIT = 2*M_PI; //TUNEME
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (TOP_LIMIT - BOTTOM_LIMIT) / (2.5/TIME_PER_CYCLE); //TUNEME
        constexpr double OPEN_LOOP_REDUC = 10.0;
    }
}