// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/ActivateShooterCmd.h"


RobotContainer::RobotContainer()
{
    ConfigureBindings();
    operatorGamepad.startShooter.OnTrue(ActivateShooterCmd(&shooterSubsystem, ShooterSubsystem::WantedState::SHOOT).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.stopShooter.OnTrue(ActivateShooterCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STOP).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}

void RobotContainer::ConfigureBindings() {
}