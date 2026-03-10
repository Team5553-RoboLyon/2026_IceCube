// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/Commands.h>

#include "commands/SetWantedSuperstructureSuperStateCmd.h"
#include "commands/ToggleMaintainPidCmd.h"

// #include "commands/SetWantedStateClimberCmd.h"
// #include "LyonLib/utils/MacroUtilsRBL.h"
// #include "commands/SetWantedShooterStateCmd.h"
// #include "commands/SetSystemTurretStateCmd.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
    drivetrain.ConfigureManualControlInputsAxis([this] { return NDEADBAND(-forwardJoystick.GetY(), driveConstants::Settings::DEADBAND); },
                                      [this] { return NDEADBAND(-rotationJoystick.GetZ(), driveConstants::Settings::DEADBAND); },
                                      [this] { return m_slowdownButton.Get(); },
                                      [this] { return 0;});

}
void RobotContainer::ConfigureBindings() {

    refuelButton.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::REFUEL)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    refuelButton.ToggleOnFalse(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::STOP_INTAKE)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    operatorGamepad.PREPARE_REFUEL.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::PREPARE_REFUEL)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.RETRACT_INTAKE.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::RETRACT_INTAKE)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));                      
    

    operatorGamepad.PREPARE_SHOOT_TO_ALLIANCE_ZONE.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::PREPARE_SHOOT_TO_ALLIANCE_ZONE)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.PREPARE_SHOOT_TO_ALLIANCE_ZONE.ToggleOnFalse(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::STAND_BY)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));


    operatorGamepad.PREPARE_SHOOT_TO_HUB.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::PREPARE_SHOOT_TO_HUB)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.PREPARE_SHOOT_TO_HUB.ToggleOnFalse(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::STAND_BY)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));


    operatorGamepad.CLIMB.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::CLIMB)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.RETRACT_CLIMBER.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::RETRACT_CLIMBER)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.PREPARE_CLIMB.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::PREPARE_CLIMB)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    
    operatorGamepad.SHOOT_TO_HUB.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::SHOOT_TO_HUB)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.SHOOT_TO_HUB.ToggleOnFalse(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::STOP_SHOOT)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    operatorGamepad.SHOOT_TO_ALLAINCE_ZONE.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::SHOOT_TO_ALLIANCE_ZONE)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    operatorGamepad.SHOOT_TO_ALLAINCE_ZONE.ToggleOnFalse(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::STOP_SHOOT)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    operatorGamepad.RETRACT_HOOD.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::RETRACT_HOOD)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    operatorGamepad.REJECT_FROM_INTAKE.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::REJECT_FROM_INTAKE)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    operatorGamepad.PROTECT_INTAKE.ToggleOnTrue(SetWantedSuperstructureSuperStateCmd(&superstructure, Superstructure::WantedSuperState::PROTECT_INTAKE)
                              .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    m_toggleMaintainPidButton.ToggleOnTrue(ToggleMaintainPidCmd(&superstructure).ToPtr());

    vision.SetDefaultCommand(vision.ProcessVision(
    [this] {
        return robotState.GetPose().value();
    },
    [this](const VisionMeasurement& measurement) {
        robotState.AddVisionMeasurement(measurement);
    }));

}