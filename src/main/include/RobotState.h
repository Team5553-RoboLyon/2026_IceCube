#pragma once

#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/filter/LinearFilter.h>
#include <frc/Timer.h>
#include <units/velocity.h>

#include "LyonLib/vision/VisionMeasurement.h"

#include "subsystems/drivetrain/DrivetrainSubsystem.h"

#include "LyonLib/logging/ComplexStructLogger.h"

#include <studica/AHRS.h>

class RobotState {
public:
    RobotState(frc::Pose2d& initialPose,
               frc::DifferentialDriveKinematics& kinematics,
               studica::AHRS& navX,
               DrivetrainSubsystem *pDrivetrain);

    void UpdateOdometry();

    void AddVisionMeasurement(const VisionMeasurement& measurement);

    void ResetPoseWithVision();
    void ResetPose(frc::Pose2d &resetPosition);

    std::optional<frc::Pose2d> GetPose() const;

private:
    studica::AHRS& m_navX;
    frc::DifferentialDrivePoseEstimator m_poseEstimator;

    StructLogger<frc::Pose2d> m_logger{"/RobotPose"};

    DrivetrainSubsystem *m_pDrivetrain;
    frc::Pose2d m_lastVisionPose;

    bool IsOdometryReliable() const;
};