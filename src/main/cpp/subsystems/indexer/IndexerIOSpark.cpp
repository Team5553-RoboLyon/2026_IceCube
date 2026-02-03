#include "subsystems/indexer/indexerIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

IndexerIOSpark::IndexerIOSpark()
{
        // Set the motor configs
    m_indexerMotorConfig.SetIdleMode(IndexerConstants::indexerMotor::IDLE_MODE)
        .Inverted(IndexerConstants::indexerMotor::INVERTED)
        .SmartCurrentLimit(IndexerConstants::indexerMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(IndexerConstants::indexerMotor::RAMP_RATE)
        .VoltageCompensation(IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_indexerMotor.Configure(  m_indexerMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_indexerMotor.ClearFaults();
    
}

void IndexerIOSpark::UpdateInputs(IndexerIOInputs& inputs) 
{
        inputs.isindexerMotorConnected = (m_indexerMotor.GetBusVoltage() !=0.0) && !m_indexerMotor.GetFaults().can;

    inputs.indexerMotorAppliedVoltage = m_indexerMotor.GetAppliedOutput() * IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION;
    inputs.indexerMotorBusVoltage = m_indexerMotor.GetBusVoltage();
    inputs.indexerMotorCurrent = m_indexerMotor.GetOutputCurrent();
    inputs.indexerMotorTemperature = m_indexerMotor.GetMotorTemperature();
    
    frc::SmartDashboard::PutNumber("indexer/IndexerVelocity", m_indexerMotor.GetEncoder().GetVelocity()/3);
}

void IndexerIOSpark::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION) 
        ,"Indexer Voltage out of range");
    
    m_indexerMotor.SetVoltage(units::volt_t(voltage));
}

void IndexerIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Indexer Duty Cycle out of range");
        m_indexerMotor.Set(dutyCycle);
}

