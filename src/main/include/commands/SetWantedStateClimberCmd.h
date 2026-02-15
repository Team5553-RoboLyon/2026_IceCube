// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/climber/ClimberSubsystem.h"


class SetWantedClimberStateCmd
    : public frc2::CommandHelper<frc2::Command, SetWantedClimberStateCmd> {
 public:
  SetWantedClimberStateCmd(ClimberSubsystem *pClimber, ClimberSubsystem::WantedState wantedState);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    ClimberSubsystem* m_pClimber;
    ClimberSubsystem::WantedState m_wantedState;
};