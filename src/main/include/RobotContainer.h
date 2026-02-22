// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"
#include "RobotState.h"

#include <frc/Joystick.h>

#include "LyonLib/Vision/Vision.h"
#include "LyonLib/Vision/VisionFilterParameters.h"
#include "LyonLib/Vision/RealPhotonVisionIO.h"
#include <frc/apriltag/AprilTagFieldLayout.h>

class RobotContainer {
 public:
  RobotContainer();

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};

  frc::AprilTagFieldLayout aprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);
  VisionFilterParameters visionFilterParameters{
    .xyStandardDevBase = 0.1,
    .rotStandardDevBase = 5.0,
    .aprilTagWidth = 0.1524_m, // 6 inches in meters
    .maxAmbiguityRatio = 0.15,
    .maxAprilTagDistance = 5.0, // meters
    .estimatedFOV = frc::Rotation2d(units::radian_t(60.0 * (M_PI / 180.0))), // 60 degrees in radians
    .zMargin = 0.5_m, // meters
    .aprilTagFieldLayout = aprilTagFieldLayout
  };

  studica::AHRS ahrs{studica::AHRS::NavXComType::kMXP_SPI, studica::AHRS::NavXUpdateRate::k100Hz};

  std::vector<std::shared_ptr<VisionIO>> visionIOs{
    std::make_shared<RealPhotonVisionIO>(
      "Big_brother",
      frc::Transform3d(
        frc::Translation3d(0.10_m, 0.0_m, 0.46_m),
        frc::Rotation3d()
      ),
      aprilTagFieldLayout
    )
  };

  Vision vision{visionFilterParameters,visionIOs};

  units::meter_t leftDistance = 0_m;
  units::meter_t rightDistance = 0_m;
  frc::Pose2d initialPose{0_m, 0_m, frc::Rotation2d()};
  frc::DifferentialDriveKinematics kinematics{0.6_m};

  RobotState robotState{
    initialPose, 
    kinematics,
    &leftDistance,
    &rightDistance,
    ahrs
    };

 private:
  void ConfigureBindings();
};