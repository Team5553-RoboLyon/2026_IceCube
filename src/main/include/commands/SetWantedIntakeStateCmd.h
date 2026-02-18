#pragma once

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"

#include "subsystems/intake/IntakeSubsystem.h"

class SetWantedIntakeStateCmd : public frc2::CommandHelper<frc2::Command, SetWantedIntakeStateCmd> {
    public:
        SetWantedIntakeStateCmd(IntakeSubsystem *pIntake, IntakeSubsystem::WantedState wantedState);

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        IntakeSubsystem* m_pIntake;
        IntakeSubsystem::WantedState m_wantedState;
};