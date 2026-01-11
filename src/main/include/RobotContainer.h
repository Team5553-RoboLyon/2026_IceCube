// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  Operator operatorGamepad{OPERATOR_GAMEPAD_PORT, OPERATOR_GAMEPAD_THRESHOLD};
 private:
  void ConfigureBindings();
};