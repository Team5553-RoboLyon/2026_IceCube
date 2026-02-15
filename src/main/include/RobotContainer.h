// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Operator.h"
#include "Constants.h"

#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/intake/roller/RollerIOSpark.h"
#include "subsystems/intake/pivot/PivotIOSpark.h"
#include <frc/Joystick.h>

#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/indexer/IndexerIOSpark.h"
#include "subsystems/intake/pivot/PivotIOSim.h"
#include "subsystems/intake/roller/RollerIOSim.h"

class RobotContainer {
 public:
  RobotContainer();
  
  Operator operatorGamepad{ControlPanelConstants::OPERATOR_GAMEPAD_PORT, ControlPanelConstants::OPERATOR_GAMEPAD_THRESHOLD};
  frc::Joystick forwardJoystick{ControlPanelConstants::JOYSTICK_FORWARD_ID};
  frc::Joystick rotationJoystick{ControlPanelConstants::JOYSTICK_ROTATION_ID};

  #if ROBOT_MODEL == SIMULATION
    // IndexerSubsystem indexer{new IndexerIOSpark{}};
    IntakeSubsystem intakeSubsystem{new RollerIOSim{}, new PivotIOSim{}};
  #else
    // IndexerSubsystem indexer{new IndexerIOSpark{}};
    IntakeSubsystem intakeSubsystem{new RollerIOSpark{}, new PivotIOSpark{}};
  #endif


 private:
  void ConfigureBindings();
};