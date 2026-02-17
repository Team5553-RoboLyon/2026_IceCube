// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetWantedIndexerStateCmd.h"

SetWantedIndexerStateCmd::SetWantedIndexerStateCmd(IndexerSubsystem *pIndexer, IndexerSubsystem::WantedState wantedState) 
: m_pIndexer(pIndexer), m_wantedState(wantedState)
{
  AddRequirements(m_pIndexer);
}

// Called when the command is initially scheduled.
void SetWantedIndexerStateCmd::Initialize() {
  m_pIndexer->SetWantedState(m_wantedState);
}

// Called repeatedly when this Command is scheduled to run
void SetWantedIndexerStateCmd::Execute() {}

// Called once the command ends or is interrupted.
void SetWantedIndexerStateCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SetWantedIndexerStateCmd::IsFinished() {
  return true;
}
