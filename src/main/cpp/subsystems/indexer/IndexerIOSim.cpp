#include "subsystems/indexer/IndexerIOSim.h"

IndexerIOSim::IndexerIOSim()
{
}

void IndexerIOSim::UpdateInputs(IndexerIOInputs &inputs)
{
    m_indexerSim.Update(2.0_ms);
    m_clodeSim.Update(2.0_ms);

    inputs.isindexerMotorConnected = true;
    inputs.isClodeMotorConnected = true;

    inputs.indexerMotorAppliedVoltage = double(m_indexerSim.GetInputVoltage());
    inputs.indexerMotorBusVoltage = 12.0;
    inputs.indexerMotorCurrent = m_indexerSim.GetCurrentDraw().value();
    inputs.indexerMotorTemperature = 24.0;

    inputs.clodeAppliedVoltage = double(m_clodeSim.GetInputVoltage());
    inputs.clodeBusVoltage = 12.0;
    inputs.clodeCurrent = m_clodeSim.GetCurrentDraw().value();
    inputs.clodeTemperature = 24.0;

    inputs.wasThereABall = inputs.isThereABall;
    inputs.isThereABall = m_IRBreakerSim.GetValue();
}

void IndexerIOSim::SetVoltage(double voltage, double clodeVoltage)
{
    DEBUG_ASSERT((voltage <= IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION) 
        ,"Indexer Voltage out of range");

    DEBUG_ASSERT((clodeVoltage <= IndexerConstants::clodeMotor::VOLTAGE_COMPENSATION) 
        && (clodeVoltage >= -IndexerConstants::clodeMotor::VOLTAGE_COMPENSATION) 
        ,"Clode Voltage out of range");

    m_indexerSim.SetInputVoltage(units::volt_t(voltage));
    m_clodeSim.SetInputVoltage(units::volt_t(clodeVoltage));
}

void IndexerIOSim::SetDutyCycle(double dutyCycle, double clodeDutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Indexer Duty Cycle out of range");

    DEBUG_ASSERT((clodeDutyCycle <= 1.0) && (clodeDutyCycle >= -1.0) 
        ,"Clode Duty Cycle out of range");

    m_indexerSim.SetInputVoltage(units::volt_t(dutyCycle*IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION));
    m_clodeSim.SetInputVoltage(units::volt_t(clodeDutyCycle*IndexerConstants::clodeMotor::VOLTAGE_COMPENSATION));
}