// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include "subsystems/drivetrain/DrivetrainSubsystem.h"
#include "subsystems/drivetrain/DrivetrainIOFlex.h"
#include "subsystems/drivetrain/DrivetrainIOSim.h"

class RobotContainer {
 public:
  RobotContainer();

  #if ROBOT_MODEL == SIMULATION
  DrivetrainSubsystem drivetrain{new DrivetrainIOSim()};
  #else
  DrivetrainSubsystem drivetrain{new DrivetrainIOFlex()};
  #endif

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};
 private:
  void ConfigureBindings();
  frc2::JoystickButton m_SlowDriveButton{&forwardJoystick, ControlPanelConstants::SLOW_DRIVE_BUTTON};
  frc2::JoystickButton m_driveActionButton{&rotationJoystick, ControlPanelConstants::ACTION_DRIVE_BUTTON};

};