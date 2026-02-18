// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"
#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/climber/ClimberIOSpark.h"
#include "subsystems/climber/ClimberIOSim.h"

#include <frc/Joystick.h>

#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/indexer/IndexerIOSpark.h"
#include "subsystems/indexer/IndexerIOSim.h"
#include "subsystems/intake/pivot/PivotIOSim.h"
#include "subsystems/intake/roller/RollerIOSim.h"

class RobotContainer {
 public:
  RobotContainer();
  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};


  #if ROBOT_MODEL == SIMULATION
  ClimberSubsystem climber{new ClimberIOSim};
  IndexerIOSim *IOSim = new IndexerIOSim{};
  IndexerSubsystem indexer{IOSim};
  IntakeSubsystem intakeSubsystem{new RollerIOSim{}, new PivotIOSim{}};
  #else
  ClimberSubsystem climber{new ClimberIOSpark};
  IndexerSubsystem indexer{new IndexerIOSpark};
  IntakeSubsystem intakeSubsystem{new RollerIOSpark{}, new PivotIOSpark{}};
  #endif


 private:
  void ConfigureBindings();
};