// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ActivateShooterCmd.h"

ActivateShooterCmd::ActivateShooterCmd(ShooterSubsystem *pShooterSubsystem, ShooterSubsystem::WantedState wantedState)
    : m_pShooterSubsystem(pShooterSubsystem), m_wantedState(wantedState) {
  AddRequirements({m_pShooterSubsystem});
}

// Called when the command is initially scheduled.
void ActivateShooterCmd::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ActivateShooterCmd::Execute() {
    m_pShooterSubsystem->SetWantedState(m_wantedState);
}

// Called once the command ends or is interrupted.
void ActivateShooterCmd::End(bool interrupted) {
  m_pShooterSubsystem->SetWantedState(ShooterSubsystem::WantedState::STAND_BY);
  
}

// Returns true when the command should end.
bool ActivateShooterCmd::IsFinished() {
  return true;
}