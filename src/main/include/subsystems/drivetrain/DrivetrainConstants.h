#pragma once
#include "Constants.h"
#include "LyonLib/control/ControlMode.h"
#include "LyonLib/logging/DebugUtils.h"
#include "rev/SparkMax.h"    
#include <cmath>

enum class DriveMode
{
    ARCADE_DRIVE,
    CURVE_DRIVE,
    AUTO_PATH_FOLLOWER,
    DISABLE
};

#if ROBOT_MODEL != COMPETITON
    #define DRIVETRAIN_SMARTDASHBOARD_LOG
#endif
using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if ROBOT_MODEL == COMPETITION || ROBOT_MODEL == TRAINING || ROBOT_MODEL == PROTOTYPE || ROBOT_MODEL == DEMO || ROBOT_MODEL == SIMULATION
namespace driveConstants 
{
    constexpr DriveMode desiredDriveControl = DriveMode::DISABLE;

    namespace Motors
    {
        //LEFT GEARBOX
        constexpr int LEFT_FRONT_MOTOR_ID = 11;
        constexpr int LEFT_BACK_MOTOR_ID = 12;
        constexpr bool LEFT_MOTOR_INVERTED = true;

        //RIGHT GEARBOX
        constexpr int RIGHT_FRONT_MOTOR_ID = 9;
        constexpr int RIGHT_BACK_MOTOR_ID = 10;
        constexpr bool RIGHT_MOTORS_INVERTED = false;

        //BOTH
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr int CURRENT_LIMIT = 40; //TUNEME ? : En fonction de la batterie
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 60;
        constexpr int OVERHEATING_THRESHOLD = 75;
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (56.0/11.0)*(42.0/18.0); //ul 
        constexpr int KV = 568.8; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motors::VOLTAGE_COMPENSATION * KV; //RPM
        
        constexpr double WHEEL_RADIUS = 2.5 * 0.0254; //m
        constexpr double TRACKWIDTH = 0.6008685; //m //TUNEME ? : A mesurer précisément ?
        constexpr double BASE_TRACK_RADIUS = TRACKWIDTH/2.0; //m

        constexpr double MAX_LINEAR_SPEED = WHEEL_RADIUS * (2.0 * NF64_PI) * ((MOTOR_FREE_SPEED / GEAR_RATIO) / 60.0); // m.s-1
        constexpr double MAX_ROTATION_SPEED = MAX_LINEAR_SPEED / BASE_TRACK_RADIUS;
        constexpr double LINEAR_TO_MOTOR_SPEED_FACTOR = (30.0 * GEAR_RATIO) / (NF64_PI * WHEEL_RADIUS); // RPM.s.m-1
    }
    namespace Encoder
    {
        constexpr int LEFT_ID_ENCODER_A = 1;
        constexpr int LEFT_ID_ENCODER_B = 2;
        constexpr bool LEFT_REVERSE_ENCODER = false; 

        constexpr int RIGHT_ID_ENCODER_A = 3;
        constexpr int RIGHT_ID_ENCODER_B = 4;
        constexpr bool RIGHT_REVERSE_ENCODER = true;

        constexpr double DISTANCE_PER_PULSE = (2.0 * NF64_PI * Specifications::WHEEL_RADIUS)/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace Simulation
    {
        constexpr double MOI = 5.23; //TUNEME
        constexpr double MASS = 60.0; //TUNEME
    }

    namespace ArcadeDrive
    {
        constexpr double MIN_ROTATION_SIGMA = 0.15; //TUNEME
        constexpr double MAX_ROTATION_SIGMA = 0.3; //TUNEME

        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.35; //TUNEME
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.4; //TUNEME

        constexpr double TIME_TO_STOP_FORWARD = 0.5; //TUNEME
        constexpr double TIME_TO_STOP_ROTATION = 0.15; //TUNEME
    }

    namespace CurveDrive //COMMENTME
    {
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_STOP_FORWARD = 0.6; //TUNEME

        constexpr double SINUSOIDAL_CURVATURE_INTENSITY = 0.5; //TUNEME
        constexpr double DENOMINATOR = 0.7071067812; // Precomputed value of sin(SINUSOIDAL_CURVATURE_INTENSITY{0.5} * M_PI_2)

        constexpr double QUICK_STOP_ALPHA = 0.1; //TUNEME
        constexpr double NEG_INERTIA_SCALAR = 4.0; //TUNEME
        constexpr double TURN_SENSITIVITY   = 1.0; //TUNEM
    }

    namespace Settings
    {
        constexpr double SLOW_RATE = 2.0; //TUNEME
        constexpr double DEADBAND = 0.05; 
    }
}
#elif ROBOT_MODEL == T_NOR
namespace driveConstants 
{
    constexpr DriveMode desiredDriveControl = DriveMode::ARCADE_DRIVE;

    namespace Motors
    {
        //LEFT GEARBOX
        constexpr int LEFT_FRONT_MOTOR_ID = 3;
        constexpr int LEFT_BACK_MOTOR_ID = 4;
        constexpr bool LEFT_MOTOR_INVERTED = true;

        //RIGHT GEARBOX
        constexpr int RIGHT_FRONT_MOTOR_ID = 1;
        constexpr int RIGHT_BACK_MOTOR_ID = 2;
        constexpr bool RIGHT_MOTORS_INVERTED = false;

        //BOTH
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr int CURRENT_LIMIT = 40;
        constexpr double RAMP_RATE = 0.0; //TUNEME
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (84.0/11.0)*(44.0/34.0); //ul
        constexpr int KV = 568.8; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motors::VOLTAGE_COMPENSATION * KV; //RPM
        
        constexpr double WHEEL_RADIUS = 3.125 * 0.0254; //m
        constexpr double TRACKWIDTH = 0.5722; //m
        constexpr double BASE_TRACK_RADIUS = TRACKWIDTH/2.0; //m

        constexpr double MAX_LINEAR_SPEED = WHEEL_RADIUS * (2.0 * NF64_PI) * ((MOTOR_FREE_SPEED / GEAR_RATIO) / 60.0); // m.s-1
        constexpr double MAX_ROTATION_SPEED = MAX_LINEAR_SPEED / BASE_TRACK_RADIUS;
        constexpr double LINEAR_TO_MOTOR_SPEED_FACTOR = (30.0 * GEAR_RATIO) / (NF64_PI * WHEEL_RADIUS); // RPM.s.m-1
    }
    namespace Encoder
    {
        constexpr int LEFT_ID_ENCODER_A = 0;
        constexpr int LEFT_ID_ENCODER_B = 1;
        constexpr bool LEFT_REVERSE_ENCODER = false; 

        constexpr int RIGHT_ID_ENCODER_A = 2;
        constexpr int RIGHT_ID_ENCODER_B = 3;
        constexpr bool RIGHT_REVERSE_ENCODER = true;

        constexpr double DISTANCE_PER_PULSE = (2.0 * NF64_PI * Specifications::WHEEL_RADIUS)/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace ArcadeDrive
    {
        constexpr double MIN_ROTATION_SIGMA = 0.05; //TUNEME
        constexpr double MAX_ROTATION_SIGMA = 0.2; //TUNEME

        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.5; //TUNEME

        constexpr double TIME_TO_STOP_FORWARD = 0.7; //TUNEME
        constexpr double TIME_TO_STOP_ROTATION = 0.35; //TUNEME
    }

    namespace CurveDrive //COMMENTME
    {
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_STOP_FORWARD = 0.6; //TUNEME

        constexpr double SINUSOIDAL_CURVATURE_INTENSITY = 0.5; //TUNEME
        constexpr double DENOMINATOR = 0.7071067812; // Precomputed value of sin(SINUSOIDAL_CURVATURE_INTENSITY{0.5} * M_PI_2)

        constexpr double QUICK_STOP_ALPHA = 0.1; //TUNEME
        constexpr double NEG_INERTIA_SCALAR = 4.0; //TUNEME
        constexpr double TURN_SENSITIVITY   = 1.0; //TUNEM
    }

    namespace Settings
    {
        constexpr double SLOW_RATE = 2.0; //TUNEME
        constexpr double DEADBAND = 0.05; 
    }
}
#elif ROBOT_MODEL == BRICE
namespace driveConstants 
{
    constexpr DriveMode desiredDriveControl = DriveMode::ARCADE_DRIVE;

    namespace Motors
    {
        //LEFT GEARBOX
        constexpr int LEFT_FRONT_MOTOR_ID = 2;
        constexpr int LEFT_BACK_MOTOR_ID = 3;
        constexpr bool LEFT_MOTOR_INVERTED = true;

        //RIGHT GEARBOX
        constexpr int RIGHT_FRONT_MOTOR_ID = 4; 
        constexpr int RIGHT_BACK_MOTOR_ID = 5;
        constexpr bool RIGHT_MOTORS_INVERTED = false;

        //BOTH
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr int CURRENT_LIMIT = 40;
        constexpr double RAMP_RATE = 0.0; //TUNEME
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (62.0/11.0) * (56.0 / 26.0); //ul
        constexpr int KV = 568.8; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motors::VOLTAGE_COMPENSATION * KV; //RPM
        
        constexpr double WHEEL_RADIUS = 2 * 0.0254; //m //TUNEME
        constexpr double TRACKWIDTH = 0.554; //m //TUNEME
        constexpr double BASE_TRACK_RADIUS = TRACKWIDTH/2.0; //m

        constexpr double MAX_LINEAR_SPEED = WHEEL_RADIUS * (2.0 * NF64_PI) * ((MOTOR_FREE_SPEED / GEAR_RATIO) / 60.0); // m.s-1
        constexpr double MAX_ROTATION_SPEED = MAX_LINEAR_SPEED / BASE_TRACK_RADIUS;
        constexpr double LINEAR_TO_MOTOR_SPEED_FACTOR = (30.0 * GEAR_RATIO) / (NF64_PI * WHEEL_RADIUS); // RPM.s.m-1
    }
    namespace Encoder
    {
        constexpr int LEFT_ID_ENCODER_A = 0; //
        constexpr int LEFT_ID_ENCODER_B = 1;
        constexpr bool LEFT_REVERSE_ENCODER = true; 

        constexpr int RIGHT_ID_ENCODER_A = 2;
        constexpr int RIGHT_ID_ENCODER_B = 3;
        constexpr bool RIGHT_REVERSE_ENCODER = false;

        constexpr double DISTANCE_PER_PULSE = (2.0 * NF64_PI * Specifications::WHEEL_RADIUS)/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace ArcadeDrive
    {
        constexpr double MIN_ROTATION_SIGMA = 0.1; //TUNEME
        constexpr double MAX_ROTATION_SIGMA = 0.45; //TUNEME

        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.5; //TUNEME

        constexpr double TIME_TO_STOP_FORWARD = 0.7; //TUNEME
        constexpr double TIME_TO_STOP_ROTATION = 0.35; //TUNEME
    }

    namespace CurveDrive //COMMENTME
    {
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_STOP_FORWARD = 0.6; //TUNEME

        constexpr double SINUSOIDAL_CURVATURE_INTENSITY = 0.5; //TUNEME
        constexpr double DENOMINATOR = 0.7071067812; // Precomputed value of sin(SINUSOIDAL_CURVATURE_INTENSITY{0.5} * NF64_PI_2)

        constexpr double QUICK_STOP_ALPHA = 0.1; //TUNEME
        constexpr double NEG_INERTIA_SCALAR = 4.0; //TUNEME
        constexpr double TURN_SENSITIVITY   = 1.0; //TUNEM
    }

    namespace Settings
    {
        constexpr double SLOW_RATE = 2.0; //TUNEME
        constexpr double DEADBAND = 0.05; 
    }
}
#else
    #error "DrivetrainConstants.h: No constants for the selected ROBOT_MODEL"
#endif