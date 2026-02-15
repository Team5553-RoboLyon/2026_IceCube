// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>
#include "commands/SetWantedIntakeStateCmd.h"


RobotContainer::RobotContainer()
{
    ConfigureBindings();
    // intakeSubsystem.SetManualControlInput([this] { return -operatorGamepad.GetLeftY(); });
}

void RobotContainer::ConfigureBindings() {
    operatorGamepad.STAND_BY.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::STAND_BY)
                                            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.BECOME_AN_INDEXER.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::BECOME_AN_INDEXER)
                                            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.EJECT.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::EJECT)
                                            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.REFUEL.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::REFUEL)
                                            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.EXTEND.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::EXTEND)
                                            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.RETURN_AT_HOME.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::RETURN_AT_HOME)
                                            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}