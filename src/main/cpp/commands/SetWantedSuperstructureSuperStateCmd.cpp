// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetWantedSuperstructureSuperStateCmd.h"

SetWantedSuperstructureSuperStateCmd::SetWantedSuperstructureSuperStateCmd(Superstructure *pSuperstructure, Superstructure::WantedSuperState wantedSuperState) 
: m_pSuperstructure(pSuperstructure), m_wantedSuperState(wantedSuperState)
{
  AddRequirements(m_pSuperstructure);
}

// Called when the command is initially scheduled.
void SetWantedSuperstructureSuperStateCmd::Initialize() {
  m_pSuperstructure->SetWantedSuperState(m_wantedSuperState);
}

// Called repeatedly when this Command is scheduled to run
void SetWantedSuperstructureSuperStateCmd::Execute() {}

// Called once the command ends or is interrupted.
void SetWantedSuperstructureSuperStateCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SetWantedSuperstructureSuperStateCmd::IsFinished() {
  return true;
}
