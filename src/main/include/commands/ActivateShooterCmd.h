// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter/ShooterSubsystem.h"


class ActivateShooterCmd
    : public frc2::CommandHelper<frc2::Command, ActivateShooterCmd> {
 public:
  ActivateShooterCmd(ShooterSubsystem *pShooterSubsystem, ShooterSubsystem::WantedState wantedState);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    ShooterSubsystem* m_pShooterSubsystem;
    ShooterSubsystem::WantedState m_wantedState;
};