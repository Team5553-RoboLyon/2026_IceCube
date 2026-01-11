// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "LyonLib/utils/MacroUtilsRBL.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
    drivetrain.ConfigureManualControlInputsAxis([this] { return NDEADBAND(-forwardJoystick.GetY(), driveConstants::Settings::DEADBAND); },
                                      [this] { return NDEADBAND(rotationJoystick.GetZ(), driveConstants::Settings::DEADBAND); },
                                      [this] { return m_SlowDriveButton.Get(); },
                                      [this] { return m_driveActionButton.Get();});
}

void RobotContainer::ConfigureBindings() {
}