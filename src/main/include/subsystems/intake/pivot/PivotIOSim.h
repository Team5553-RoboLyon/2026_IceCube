#pragma once

#include "PivotIO.h"
#include "PivotConstants.h"

#include "frc/simulation/SingleJointedArmSim.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/simulation/DutyCycleEncoderSim.h"

class PivotIOSim final : public PivotIO 
{
    private:
        frc::DCMotor m_pivotSimMotor{frc::DCMotor::NEO(1)};

        frc::sim::SingleJointedArmSim m_pivotSim{m_pivotSimMotor,
                                                 PivotConstants::Specifications::PIVOT_GEAR_RATIO,
                                                 PivotConstants::Simulation::MOI,
                                                 PivotConstants::Simulation::ARM_LENGTH,
                                                 units::radian_t(PivotConstants::Position::MIN),
                                                 units::radian_t(PivotConstants::Position::MAX),
                                                 PivotConstants::Simulation::APPLY_GRAVITY,
                                                 units::radian_t(PivotConstants::Position::PIVOT_HOME_POS)
                                                };

        // frc::sim::DutyCycleEncoderSim m_leftEncoder{PivotConstants::EncoderLeft::ID};
        // frc::sim::DutyCycleEncoderSim m_rightEncoder{PivotConstants::EncoderRight::ID};

    public:
        PivotIOSim();
        ~PivotIOSim() = default;

        void UpdateInputs(PivotIOInputs& inputs) override;
        void SetVoltage(double voltage) override;
        void SetDutyCycle(double dutyCycle) override;
};
