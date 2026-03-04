// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"
#include "RobotState.h"

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <functional>

#if ROBOT_MODEL == SIMULATION
  #include "subsystems/shooter/flywheel/FlywheelIOSim.h"
  #include "subsystems/shooter/hood/HoodIOSim.h"
  #include "subsystems/turret/TurretIOSim.h"
  #include "subsystems/climber/ClimberIOSim.h"
  #include "subsystems/intake/roller/RollerIOSim.h"
  #include "subsystems/intake/pivot/PivotIOSim.h"
  #include "subsystems/drivetrain/DrivetrainIOSim.h"
  #include "subsystems/turret/TurretIOSim.h"
#else
  #include "subsystems/shooter/flywheel/FlywheelIOSpark.h"
  #include "subsystems/shooter/hood/HoodIOSpark.h"
  #include "subsystems/turret/TurretIOSpark.h"
  #include "subsystems/climber/ClimberIOSpark.h"
  #include "subsystems/intake/roller/RollerIOSpark.h"
  #include "subsystems/intake/pivot/PivotIOSpark.h"
  #include "subsystems/drivetrain/DrivetrainIOFlex.h"
  #include "subsystems/turret/TurretIOSpark.h"
  #include "subsystems/indexer/IndexerIOSpark.h"
#endif

#include "subsystems/superstrucure/Superstructure.h"
#include "subsystems/shooter/ShooterSubsystem.h"
#include "subsystems/ShootParametersCalculator.h"
#include "subsystems/turret/TurretSubsystem.h"
#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/drivetrain/DrivetrainSubsystem.h"
#include "subsystems/turret/TurretSubsystem.h"
#include "subsystems/indexer/IndexerSubsystem.h"




#include "LyonLib/Vision/Vision.h"
#include "LyonLib/Vision/VisionFilterParameters.h"
#include "LyonLib/Vision/RealPhotonVisionIO.h"
#include <frc/apriltag/AprilTagFieldLayout.h>

class RobotContainer {
 public:
  RobotContainer();
  ShootParameters* pShootParameter{new ShootParameters};
  ShootParametersCalculator ShootParamCalculator{};
  double robotOrientation{0.0};

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};

  ShootParameters *pShootParams{new ShootParameters};

  #if ROBOT_MODEL == SIMULATION
  IndexerIOSim *indexerIOSim = new IndexerIOSim{};
  Superstructure superstructure{new IntakeSubsystem {new RollerIOSim{}, new PivotIOSim{}},
                                new IndexerSubsystem {indexerIOSim},
                                new TurretSubsystem {new TurretIOSim, pShootParams},
                                new ShooterSubsystem {new FlywheelIOSim{}, new HoodIOSim{}, pShootParams},
                                new ClimberSubsystem {new ClimberIOSim},
                                pShootParams};
  DrivetrainSubsystem drivetrain{new DrivetrainIOSim()};
  #else
  Superstructure superstructure{new IntakeSubsystem {new RollerIOSpark{}, new PivotIOSpark{}},
                                new IndexerSubsystem {new IndexerIOSpark{}},
                                new TurretSubsystem {new TurretIOSpark{}, pShootParams},
                                new ShooterSubsystem {new FlywheelIOSpark{}, new HoodIOSpark{}, pShootParams},
                                new ClimberSubsystem {new ClimberIOSpark},
                                pShootParams};
  DrivetrainSubsystem drivetrain{new DrivetrainIOFlex()};
  #endif


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
  ),

  std::make_shared<RealPhotonVisionIO>(
    "Lil'Bro",
    frc::Transform3d(
      frc::Translation3d(-0.15_m, 0.20_m, 0.50_m),  // Position relative au centre robot
      frc::Rotation3d(0_deg, 0_deg, 180_deg)        // Orientation si caméra arrière
    ),
    aprilTagFieldLayout
  )
};

  Vision vision{visionFilterParameters,visionIOs};
  frc::Pose2d initialPose{0_m, 0_m, frc::Rotation2d()};
  frc::DifferentialDriveKinematics kinematics{
    units::meter_t(driveConstants::Specifications::TRACKWIDTH)};


  RobotState robotState{
    initialPose,
    kinematics,
    ahrs,
    &drivetrain
  };

 private:
  void ConfigureBindings();
  frc2::JoystickButton m_SlowDriveButton{&forwardJoystick, ControlPanelConstants::SLOW_DRIVE_BUTTON};
  frc2::JoystickButton m_driveActionButton{&rotationJoystick, ControlPanelConstants::ACTION_DRIVE_BUTTON};

};