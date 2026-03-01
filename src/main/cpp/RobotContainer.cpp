// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/Commands.h>

#include "commands/SetWantedShooterStateCmd.h"
#include "commands/SetSystemTurretStateCmd.h"


RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    operatorGamepad.STAND_BY.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STAND_BY),
                                                                         SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)});

    operatorGamepad.SHOOT_TO_HUB.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::SHOOT_TO_HUB),
                                                SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::FOLLOW_HUB)});

    // operatorGamepad.FEED_ALLY.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::FEED_ALLY),
    //                                                                       SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::PREPARE_EJECT)});

    operatorGamepad.STOP.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STOP),
                                                                         SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)});

    // operatorGamepad.KEEP_ALL_FOR_YOU.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::KEEP_ALL_FOR_YOU),
    //                                                                              SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE)});
                                                                                 
    operatorGamepad.REVERSE.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::REVERSE),
                                                                         SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)});
}