// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/SetSystemTurretStateCmd.h"


RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    operatorGamepad.STAND_BY.ToggleOnTrue(SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.FOLLOW_HUB.ToggleOnTrue(SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::FOLLOW_HUB)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.POINT_AT_ALLIANCE_ZONE.ToggleOnTrue(SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.PREPARE_EJECT.ToggleOnTrue(SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::PREPARE_EJECT)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}