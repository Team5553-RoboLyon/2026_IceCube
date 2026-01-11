#include "LyonLib/control/AntiTipping.h"
#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/MacroUtilsRBL.h"


AntiTipping::AntiTipping(std::function<units::angle::radian_t()> pitchSupplier,
                std::function<units::angle::radian_t()> rollSupplier,
                DriveType type,
                double kP,
                units::angle::radian_t tippingThreshold,
                units::velocity::meters_per_second_t maxCorrectionSpeed) :
                        pitchSupplier(pitchSupplier),
                        rollSupplier(rollSupplier),
                        driveType(type),
                        kP(kP),
                        tippingThreshold(tippingThreshold),
                        maxCorrectionSpeed(maxCorrectionSpeed) 
    {
        DEBUG_ASSERT(kP > 0.0, "AntiTipping: kP must be positive");
        DEBUG_ASSERT(tippingThreshold.value() > 0.0, "AntiTipping: tippingThreshold must be positive");
        DEBUG_ASSERT(maxCorrectionSpeed.value() > 0.0, "AntiTipping: maxCorrectionSpeed must be positive");
        if(driveType == DriveType::Tank) {
            DEBUG_LOG("AntiTipping: Using Tank drive type, roll will be ignored");
        }
    }

void AntiTipping::SetTippingThreshold(units::angle::radian_t angle)
{ 
    if(angle.value() >0.0) 
    {
        tippingThreshold = angle; 
    }
    else 
    {
        DEBUG_ASSERT(false, "AntiTipping: tippingThreshold must be positive");
    }
}

void AntiTipping::SetMaxCorrectionSpeed(units::velocity::meters_per_second_t velocity)
{ 
    if(velocity.value() >0.0) 
    {
        maxCorrectionSpeed = velocity; 
    }
    else 
    {
        DEBUG_ASSERT(false, "AntiTipping: maxCorrectionSpeed must be positive");
    }
}

frc::ChassisSpeeds AntiTipping::Calculate()
{
    pitch = pitchSupplier();
    roll = rollSupplier();

    switch (driveType)
    {
    case DriveType::Swerve:
        // Swerve : pitch + roll
        isTippingFlag = NABS(pitch.value()) > tippingThreshold.value() ||
                        NABS(roll.value())  > tippingThreshold.value();
        break;
    case DriveType::Tank:
        // Tank : Roll ignored
        isTippingFlag = NABS(pitch.value()) > tippingThreshold.value();
        break;
    
    default:
        DEBUG_ASSERT(false, "AntiTipping: Unknown DriveType");
        break;
    }

    if (!isTippingFlag) {
        speeds = frc::ChassisSpeeds(units::meters_per_second_t(0.0),
                                    units::meters_per_second_t(0.0),
                                    units::radians_per_second_t (0.0));
        return speeds;
    }

    switch (driveType)
    {
    case DriveType::Swerve:
        // direction of the fall
        tiltDirection = frc::Rotation2d(units::angle::radian_t(std::atan2(-roll.value(), -pitch.value())));
        yawDirection = tiltDirection.Radians();

        // magnitude : hypot(pitch, roll)
        inclinationMagnitude = units::angle::radian_t(NHYPOT(pitch.value(), roll.value()));

        // correction proportionnelle
        correctionSpeed = units::velocity::meters_per_second_t(kP * -inclinationMagnitude.value());
        correctionSpeed = units::velocity::meters_per_second_t(NCLAMP(-maxCorrectionSpeed,
                                    correctionSpeed,
                                    maxCorrectionSpeed));

        // vector toward the fall direction
        correctionVector =
            frc::Translation2d(0_m, 1_m)
                .RotateBy(tiltDirection)
                * correctionSpeed.value();

        // WPILib : Y axis inverted
        speeds = frc::ChassisSpeeds(
            units::velocity::meters_per_second_t(correctionVector.X().value()),
            units::velocity::meters_per_second_t(-correctionVector.Y().value()),
            units::angular_velocity::radians_per_second_t(0.0)
        );
        break;
    
    case DriveType::Tank:
        // inclined along pitch only
        inclinationMagnitude = units::angle::radian_t(NABS(pitch.value()));

        correctionSpeed = units::velocity::meters_per_second_t(kP * -pitch.value());
        correctionSpeed = NCLAMP(-maxCorrectionSpeed,
                                    correctionSpeed,
                                    maxCorrectionSpeed);

        // Tank : move forward/backward only
        speeds = frc::ChassisSpeeds(
            correctionSpeed, // vx
            units::velocity::meters_per_second_t(0.0), // vy = impossible
            units::angular_velocity::radians_per_second_t(0.0) // no rotation
        );

        // tiltDirection & yawDirection : optional
        tiltDirection = frc::Rotation2d(0_rad);
        yawDirection = 0_rad;
        break;
    default:
        DEBUG_ASSERT(false, "AntiTipping: Unknown DriveType");
        break;
    }

    return speeds;
}


units::angle::radian_t AntiTipping::GetPitch() const { return pitch; }
units::angle::radian_t AntiTipping::GetRoll()  const { return roll; }
bool   AntiTipping::IsTipping() const { return isTippingFlag; }

units::angle::radian_t AntiTipping::GetInclinationMagnitude() const { return inclinationMagnitude; }
units::angle::radian_t AntiTipping::GetYawDirection() const { return yawDirection; }

frc::Rotation2d AntiTipping::GetTiltDirection() const { return tiltDirection; }
frc::ChassisSpeeds AntiTipping::GetVelocityAntiTipping() const { return speeds; }