// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/SetWantedStateClimberCmd.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

    operatorGamepad.CLIMBED.OnTrue(SetWantedClimberStateCmd(&climber, ClimberSubsystem::WantedState::CLIMBED)
                                .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.ARMED_TO_CLIMB.OnTrue(SetWantedClimberStateCmd(&climber, ClimberSubsystem::WantedState::ARMED_TO_CLIMB)
                                .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.STOWED.OnTrue(SetWantedClimberStateCmd(&climber, ClimberSubsystem::WantedState::STOWED)
                                .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.toggle.OnTrue(frc2::InstantCommand([this](){climber.ToggleControlMode();})
                                .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}