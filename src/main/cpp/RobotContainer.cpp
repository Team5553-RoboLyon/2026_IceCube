// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/SetIRBreakerValueCmd.h"
#include "commands/SetWantedIndexerStateCmd.h"


RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    operatorGamepad.STAND_BY.ToggleOnTrue(SetWantedIndexerStateCmd(&indexer, IndexerSubsystem::WantedState::STAND_BY)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.PREPARE_SHOOT.ToggleOnTrue(SetWantedIndexerStateCmd(&indexer, IndexerSubsystem::WantedState::PREPARE_SHOOT)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.EVACUATE_SHOOTER.ToggleOnTrue(SetWantedIndexerStateCmd(&indexer, IndexerSubsystem::WantedState::EVACUATE_SHOOTER)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.FEED_SHOOTER.ToggleOnTrue(SetWantedIndexerStateCmd(&indexer, IndexerSubsystem::WantedState::FEED_SHOOTER)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.SET_IRBREAKER_TRUE.ToggleOnTrue(SetIRBreakerValueCmd(IOSim, true)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.SET_IRBREAKER_FALSE.ToggleOnTrue(SetIRBreakerValueCmd(IOSim, false)
                                          .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}