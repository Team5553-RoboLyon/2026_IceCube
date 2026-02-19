// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetSystemTurretStateCmd.h"

SetSystemTurretStateCmd::SetSystemTurretStateCmd(TurretSubsystem *pTurret, TurretSubsystem::WantedState wantedState) 
: m_pTurret(pTurret), m_wantedState(wantedState)
{
  AddRequirements(m_pTurret);
}

// Called when the command is initially scheduled.
void SetSystemTurretStateCmd::Initialize() {
  m_pTurret->SetWantedState(m_wantedState);
}

// Called repeatedly when this Command is scheduled to run
void SetSystemTurretStateCmd::Execute() {}

// Called once the command ends or is interrupted.
void SetSystemTurretStateCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SetSystemTurretStateCmd::IsFinished() {
  return true;
}
