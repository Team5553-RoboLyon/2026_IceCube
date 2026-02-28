// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"
#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/climber/ClimberIOSpark.h"
#include "subsystems/climber/ClimberIOSim.h"

#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/intake/roller/RollerIOSpark.h"
#include "subsystems/intake/pivot/PivotIOSpark.h"
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include "subsystems/drivetrain/DrivetrainSubsystem.h"
#include "subsystems/drivetrain/DrivetrainIOFlex.h"
#include "subsystems/drivetrain/DrivetrainIOSim.h"

#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/indexer/IndexerIOSpark.h"
#include "subsystems/intake/pivot/PivotIOSim.h"
#include "subsystems/intake/roller/RollerIOSim.h"

class RobotContainer {
 public:
  RobotContainer();

  #if ROBOT_MODEL == SIMULATION
  ClimberSubsystem climber{new ClimberIOSim};
  // IndexerSubsystem indexer{new IndexerIOSpark{}};
  DrivetrainSubsystem drivetrain{new DrivetrainIOSim()};
  IntakeSubsystem intakeSubsystem{new RollerIOSim{}, new PivotIOSim{}};
  #else
  ClimberSubsystem climber{new ClimberIOSpark};
  DrivetrainSubsystem drivetrain{new DrivetrainIOFlex()};
  // IndexerSubsystem indexer{new IndexerIOSpark{}};
    IntakeSubsystem intakeSubsystem{new RollerIOSpark{}, new PivotIOSpark{}};
  #endif

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};

 private:
  void ConfigureBindings();
  frc2::JoystickButton m_SlowDriveButton{&forwardJoystick, ControlPanelConstants::SLOW_DRIVE_BUTTON};
  frc2::JoystickButton m_driveActionButton{&rotationJoystick, ControlPanelConstants::ACTION_DRIVE_BUTTON};

};