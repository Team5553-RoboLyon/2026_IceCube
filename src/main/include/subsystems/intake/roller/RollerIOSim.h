#pragma once

#include "RollerIO.h"
#include "RollerConstants.h"

#include "frc/simulation/FlywheelSim.h"
#include "frc/system/plant/LinearSystemId.h"

class RollerIOSim final : public RollerIO
{
    private:
        frc::DCMotor m_rollerMotorSim{frc::DCMotor::NEO(1)};

        frc::sim::FlywheelSim m_rollerSim{frc::LinearSystemId::FlywheelSystem(m_rollerMotorSim,
                                                                              RollerConstants::Simulation::MOI,
                                                                              RollerConstants::Specifications::ROLLER_GEAR_RATIO),
                                          m_rollerMotorSim};

    public:
        RollerIOSim();
        ~RollerIOSim() = default;

        void UpdateInputs(RollerIOInputs& inputs) override;
        void SetVoltage(double voltage) override;
        void SetDutyCycle(double dutyCycle) override;
};