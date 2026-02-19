// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetWantedShooterStateCmd.h"

SetWantedShooterStateCmd::SetWantedShooterStateCmd(ShooterSubsystem* pShooter, ShooterSubsystem::WantedState wantedState) 
 : m_pShooter(pShooter), m_wantedState(wantedState)
{
  AddRequirements(m_pShooter);
}

// Called when the command is initially scheduled.
void SetWantedShooterStateCmd::Initialize() 
{
  m_pShooter->SetWantedState(m_wantedState);
}

void SetWantedShooterStateCmd::Execute()
{

}

void SetWantedShooterStateCmd::End(bool interrupted)
{

}

bool SetWantedShooterStateCmd::IsFinished()
{
  return true;
}
