#include "commands/SetWantedIntakeStateCmd.h"

SetWantedIntakeStateCmd::SetWantedIntakeStateCmd(IntakeSubsystem *pIntake, IntakeSubsystem::WantedState wantedState)
    : m_pIntake(pIntake), m_wantedState(wantedState)
{
    AddRequirements(m_pIntake);
}

void SetWantedIntakeStateCmd::Initialize()
{
    m_pIntake->SetWantedState(m_wantedState);
}

void SetWantedIntakeStateCmd::Execute()
{}

void SetWantedIntakeStateCmd::End(bool interrupted)
{}

bool SetWantedIntakeStateCmd::IsFinished()
{
    return true;
}