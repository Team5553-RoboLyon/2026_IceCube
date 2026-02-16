// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/SetWantedShooterStateCmd.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    operatorGamepad.STAND_BY.ToggleOnTrue(SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STAND_BY)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.SHOOT_TO_HUB.ToggleOnTrue(SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::SHOOT_TO_HUB)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.FEED_ALLY.ToggleOnTrue(SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::FEED_ALLY)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.STOP.ToggleOnTrue(SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STOP)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.REVERSE.ToggleOnTrue(SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::REVERSE)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.KEEP_ALL_FOR_YOU.ToggleOnTrue(SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::KEEP_ALL_FOR_YOU)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}