// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/shooter/ShooterSubsystem.h"

class SetWantedShooterStateCmd
    : public frc2::CommandHelper<frc2::Command,
                                 SetWantedShooterStateCmd> {
 public:
  SetWantedShooterStateCmd(ShooterSubsystem* pShooter, ShooterSubsystem::WantedState wantedState);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
   ShooterSubsystem* m_pShooter;
   ShooterSubsystem::WantedState m_wantedState;
};
