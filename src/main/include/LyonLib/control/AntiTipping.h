#pragma once
/*******************************************************************************
 * 
 * File        : AntiTipping.h (v0.1)
 * Library     : LyonLib 
 * Description : Defines various control modes used in the robot's state machine 
 *               and manual control.
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 *                                and inspired by Team 9047
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

#pragma once

#include "units/angle.h"
#include "units/velocity.h"

#include <functional>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

class AntiTipping {
public:
    enum class DriveType {
        Swerve,
        Tank
    };

private:
    std::function<units::angle::radian_t()> pitchSupplier;
    std::function<units::angle::radian_t()> rollSupplier;

    DriveType driveType;
    double kP; // proportional gain
    units::angle::radian_t tippingThreshold;
    units::velocity::meters_per_second_t maxCorrectionSpeed;

    // Internal state
    units::angle::radian_t pitch = units::angle::radian_t{0.0};
    units::angle::radian_t roll = units::angle::radian_t{0.0};
    units::velocity::meters_per_second_t correctionSpeed = units::velocity::meters_per_second_t{0.0};
    units::angle::radian_t inclinationMagnitude = units::angle::radian_t{0.0};
    units::angle::radian_t yawDirection = units::angle::radian_t{0.0};
    bool isTippingFlag = false;

    frc::Rotation2d tiltDirection;
    frc::Translation2d correctionVector;
    frc::ChassisSpeeds speeds;

public:
    AntiTipping(std::function<units::angle::radian_t()> pitchSupplier,
                std::function<units::angle::radian_t()> rollSupplier,
                DriveType type,
                double kP,
                units::angle::radian_t tippingThreshold,
                units::velocity::meters_per_second_t maxCorrectionSpeed);

    void SetTippingThreshold(units::angle::radian_t angle);
    void SetMaxCorrectionSpeed(units::velocity::meters_per_second_t velocity);

    frc::ChassisSpeeds Calculate();

    units::angle::radian_t GetPitch() const;
    units::angle::radian_t GetRoll()  const;
    bool   IsTipping() const;

    units::angle::radian_t GetInclinationMagnitude() const;
    units::angle::radian_t GetYawDirection() const;

    frc::Rotation2d GetTiltDirection() const;
    frc::ChassisSpeeds GetVelocityAntiTipping() const;
};