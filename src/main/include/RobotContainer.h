// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

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

#include "subsystems/shooter/ShooterSubsystem.h"
#include "subsystems/ShootParametersCalculator.h"
#include "subsystems/turret/TurretSubsystem.h"
#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/drivetrain/DrivetrainSubsystem.h"
#include "subsystems/turret/TurretSubsystem.h"
#include "subsystems/indexer/IndexerSubsystem.h"




class RobotContainer {
 public:
  RobotContainer();
  ShootParameters* pShootParameter{new ShootParameters};
  ShootParametersCalculator ShootParamCalculator{};
  double robotOrientation{0.0};

  #if ROBOT_MODEL == SIMULATION
  ShooterSubsystem shooterSubsystem{new FlywheelIOSim{}, new HoodIOSim{}, pShootParameter};
  TurretSubsystem turretSubsystem{new TurretIOSim(), pShootParameter};
  ClimberSubsystem climber{new ClimberIOSim()};
  IndexerSubsystem indexer{new IndexerIOSim()};
  DrivetrainSubsystem drivetrain{new DrivetrainIOSim()};
  IntakeSubsystem intakeSubsystem{new RollerIOSim(), new PivotIOSim()};
  #else
  ShooterSubsystem shooterSubsystem{new FlywheelIOSpark(), new HoodIOSpark(), pShootParameter};
  TurretSubsystem turretSubsystem{new TurretIOSpark(), pShootParameter};
  ClimberSubsystem climber{new ClimberIOSpark()};
  IndexerSubsystem indexer{new IndexerIOSpark()};
  DrivetrainSubsystem drivetrain{new DrivetrainIOFlex()};
  IntakeSubsystem intakeSubsystem{new RollerIOSpark(), new PivotIOSpark()};
  #endif
  

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};

 private:
  void ConfigureBindings();
  frc2::JoystickButton m_SlowDriveButton{&forwardJoystick, ControlPanelConstants::SLOW_DRIVE_BUTTON};
  frc2::JoystickButton m_driveActionButton{&rotationJoystick, ControlPanelConstants::ACTION_DRIVE_BUTTON};

};