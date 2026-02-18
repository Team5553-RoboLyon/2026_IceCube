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
#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/climber/ClimberIOSpark.h"
#include "subsystems/climber/ClimberIOSim.h"

#include <frc/Joystick.h>

#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/indexer/IndexerIOSpark.h"
#include "subsystems/indexer/IndexerIOSim.h"
#include "subsystems/intake/pivot/PivotIOSim.h"
#include "subsystems/intake/roller/RollerIOSim.h"
#include "subsystems/turret/TurretIOSpark.h"
#include "subsystems/turret/TurretIOSim.h"
#include "subsystems/turret/TurretSubsystem.h"

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
  ShooterSubsystem shooterSubsystem{new FlywheelIOSim{}, new HoodIOSim{}, new ShootParameters{}};
  TurretSubsystem turretSubsystem{new TurretIOSim, new ShootParameters};
  #else
  ClimberSubsystem climber{new ClimberIOSpark};
  IndexerSubsystem indexer{new IndexerIOSpark};
  IntakeSubsystem intakeSubsystem{new RollerIOSpark{}, new PivotIOSpark{}};
  ShooterSubsystem shooterSubsystem{new FlywheelIOSpark{}, new HoodIOSpark{}, new ShootParameters{}};
  TurretSubsystem turretSubsystem{new TurretIOSpark, new ShootParameters};
  #endif


 private:
  void ConfigureBindings();
};