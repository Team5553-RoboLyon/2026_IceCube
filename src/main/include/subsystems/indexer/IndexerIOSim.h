#pragma once

#include "IndexerIO.h"
#include "IndexerConstants.h"

#include "frc/simulation/FlywheelSim.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/simulation/DIOSim.h"

#include "LyonLib/logging/TunableValueLogger.h"

#include "units/moment_of_inertia.h"

class IndexerIOSim final : public IndexerIO
{
    private:
    
     frc::DCMotor m_indexerMotorSim{frc::DCMotor::NEO(1)};
     frc::DCMotor m_clodeMotorSim{frc::DCMotor::NEO550(1)};

     frc::sim::FlywheelSim m_indexerSim{frc::LinearSystemId::FlywheelSystem(m_indexerMotorSim,
                                                                            units::kilogram_square_meter_t(IndexerConstants::Simulation::MOI), 
                                                                            IndexerConstants::Specifications::GEAR_RATIO),
                                        m_indexerMotorSim};

     frc::sim::FlywheelSim m_clodeSim{frc::LinearSystemId::FlywheelSystem(m_clodeMotorSim, 
                                                                          units::kilogram_square_meter_t(IndexerConstants::Simulation::CLODE_MOI),
                                                                          IndexerConstants::clodeMotor::GEAR_RATIO),
                                      m_clodeMotorSim};

    public:

     IndexerIOSim();
     ~IndexerIOSim() = default;

     void UpdateInputs(IndexerIOInputs &inputs) override;

     void SetVoltage(double voltage, double clodeVoltage) override;
     void SetDutyCycle(double dutyCycle, double clodeDutyCycle) override;

     frc::sim::DIOSim m_IRBreakerSim{0};
};