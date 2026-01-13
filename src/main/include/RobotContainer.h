// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"

#include <frc/Joystick.h>

#include "subsystems/turret/TurretIOSpark.h"
#include "subsystems/turret/TurretSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  TurretSubsystem turretSubsystem{new TurretIOSpark()};

  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};
 private:
  void ConfigureBindings();
};