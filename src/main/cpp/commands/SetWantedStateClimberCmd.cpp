// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetWantedStateClimberCmd.h"

SetWantedClimberStateCmd::SetWantedClimberStateCmd(ClimberSubsystem *pClimber, ClimberSubsystem::WantedState wantedState)
    : m_pClimber(pClimber), m_wantedState(wantedState) {
  AddRequirements({m_pClimber});
}

// Called when the command is initially scheduled.
void SetWantedClimberStateCmd::Initialize() {
  m_pClimber->SetWantedState(m_wantedState);
}

// Called repeatedly when this Command is scheduled to run
void SetWantedClimberStateCmd::Execute() {

}

// Called once the command ends or is interrupted.
void SetWantedClimberStateCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool SetWantedClimberStateCmd::IsFinished() {
  return true;
}