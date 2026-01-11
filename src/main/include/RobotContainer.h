// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/shooter/ShooterSubsystem.h"

#include "subsystems/shooter/ShooterIOSpark.h"
#include "subsystems/Operator.h"
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  ShooterSubsystem m_ShooterSubsystem{new ShooterIOSpark()};
  Operator operatorGamepad{OPERATOR_GAMEPAD_PORT, OPERATOR_GAMEPAD_THRESHOLD};
 private:
  void ConfigureBindings();
};