// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/shooter/ShooterSubsystem.h"

#include "subsystems/shooter/ShooterIOSpark.h"

class RobotContainer {
 public:
  RobotContainer();

  ShooterSubsystem m_ShooterSubsystem{new ShooterIOSpark()};
 private:

  void ConfigureBindings();
};