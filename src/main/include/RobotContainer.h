// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/shooter/ShooterSubsystem.h"

#include "subsystems/shooter/flywheel/FlywheelIOSpark.h"
#include "subsystems/shooter/hood/HoodIOSpark.h"
#include "subsystems/shooter/hood/HoodIOSim.h"
#include "subsystems/shooter/flywheel/FlywheelIOSim.h"
#include "subsystems/Operator.h"
#include "Constants.h"

#include <frc/Joystick.h>

class RobotContainer {
 public:
  RobotContainer();

  #if ROBOT_MODEL == SIMULATION
  ShooterSubsystem shooterSubsystem{new FlywheelIOSim{}, new HoodIOSim{}, new ShootParameters{}};
  #else
  ShooterSubsystem shooterSubsystem{new FlywheelIOSpark{}, new HoodIOSpark{}, new ShootParameters{}};
  #endif

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};
 private:
  void ConfigureBindings();
};