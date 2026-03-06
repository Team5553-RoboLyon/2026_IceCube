// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/Commands.h>
#include "commands/SetWantedIntakeStateCmd.h"

#include "commands/SetWantedStateClimberCmd.h"
#include "LyonLib/utils/MacroUtilsRBL.h"
#include "commands/SetWantedShooterStateCmd.h"
#include "commands/SetSystemTurretStateCmd.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
    drivetrain.ConfigureManualControlInputsAxis([this] { return NDEADBAND(-forwardJoystick.GetY(), driveConstants::Settings::DEADBAND); },
                                      [this] { return NDEADBAND(-rotationJoystick.GetZ(), driveConstants::Settings::DEADBAND); },
                                      [this] { return m_SlowDriveButton.Get(); },
                                      [this] { return m_driveActionButton.Get();});

}
void RobotContainer::ConfigureBindings() {
    // operatorGamepad.STAND_BY.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::STAND_BY)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.BECOME_AN_INDEXER.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::BECOME_AN_INDEXER)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.EJECT.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::EJECT)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.REFUEL.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::REFUEL)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.EXTEND.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::EXTEND)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.RETURN_AT_HOME.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::RETURN_AT_HOME)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.PROTECT_YOURSELF.ToggleOnTrue(SetWantedIntakeStateCmd(&intakeSubsystem, IntakeSubsystem::WantedState::PROTECT_YOURSELF_AGAINST_EVIL_PILOT)
    //                                         .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // operatorGamepad.CLIMBED.OnTrue(SetWantedClimberStateCmd(&climber, ClimberSubsystem::WantedState::CLIMBED)
    //                             .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.ARMED_TO_CLIMB.OnTrue(SetWantedClimberStateCmd(&climber, ClimberSubsystem::WantedState::ARMED_TO_CLIMB)
    //                             .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.STOWED.OnTrue(SetWantedClimberStateCmd(&climber, ClimberSubsystem::WantedState::STOWED)
    //                             .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // operatorGamepad.toggle.OnTrue(frc2::InstantCommand([this](){climber.ToggleControlMode();})
    //                             .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    // intakeSubsystem.SetManualControlInput([this] { return -operatorGamepad.GetLeftY(); });
    // operatorGamepad.STAND_BY.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STAND_BY),
    //                                                                      SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)});

    // operatorGamepad.SHOOT_TO_HUB.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::SHOOT_TO_HUB),
    //                                             SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::FOLLOW_HUB)});

    // // operatorGamepad.FEED_ALLY.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::FEED_ALLY),
    // //                                                                       SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::PREPARE_EJECT)});

    // operatorGamepad.STOP.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::STOP),
    //                                                                      SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)});

    // // operatorGamepad.KEEP_ALL_FOR_YOU.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::KEEP_ALL_FOR_YOU),
    // //                                                                              SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE)});
                                                                                 
    // operatorGamepad.REVERSE.ToggleOnTrue(new frc2::ParallelCommandGroup{SetWantedShooterStateCmd(&shooterSubsystem, ShooterSubsystem::WantedState::REVERSE),
                                                                        //  SetSystemTurretStateCmd(&turretSubsystem, TurretSubsystem::WantedState::STAND_BY)});
    vision.SetDefaultCommand(vision.ProcessVision(
    [this] {
        return robotState.GetPose().value();
    },
    [this](const VisionMeasurement& measurement) {
        robotState.AddVisionMeasurement(measurement);
    }));

}