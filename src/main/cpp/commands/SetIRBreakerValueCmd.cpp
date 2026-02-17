// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetIRBreakerValueCmd.h"

SetIRBreakerValueCmd::SetIRBreakerValueCmd(IndexerIOSim* pIndexerSim, bool value) : m_pIndexerSim(pIndexerSim), m_value(value)  {
  
}

// Called when the command is initially scheduled.
void SetIRBreakerValueCmd::Initialize() {
  m_pIndexerSim->m_IRBreakerSim.SetValue(m_value);
}

// Called repeatedly when this Command is scheduled to run
void SetIRBreakerValueCmd::Execute() {}

// Called once the command ends or is interrupted.
void SetIRBreakerValueCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SetIRBreakerValueCmd::IsFinished() {
  return true;
}
