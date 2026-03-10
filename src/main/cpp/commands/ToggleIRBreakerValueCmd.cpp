// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToggleIRBreakerValueCmd.h"

ToggleIRBreakerValueCmd::ToggleIRBreakerValueCmd(IndexerIOSim* pIndexerSim) : m_pIndexerSim(pIndexerSim){
  
}

// Called when the command is initially scheduled.
void ToggleIRBreakerValueCmd::Initialize() {
  m_pIndexerSim->m_IRBreakerSim.SetValue(!m_pIndexerSim->m_IRBreakerSim.GetValue());
}

// Called repeatedly when this Command is scheduled to run
void ToggleIRBreakerValueCmd::Execute() {}

// Called once the command ends or is interrupted.
void ToggleIRBreakerValueCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ToggleIRBreakerValueCmd::IsFinished() {
  return true;
}
