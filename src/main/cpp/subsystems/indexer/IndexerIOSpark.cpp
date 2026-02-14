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
        .VoltageCompensation(IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION)
        .closedLoop.P(IndexerConstants::Gains::VELOCITY_DUTYCYCLE_PIDF::KP)
        .I(IndexerConstants::Gains::VELOCITY_DUTYCYCLE_PIDF::KI)
        .D(IndexerConstants::Gains::VELOCITY_DUTYCYCLE_PIDF::KD)
        .feedForward.kV(IndexerConstants::Gains::VELOCITY_DUTYCYCLE_PIDF::KF);

    // Apply the configs to the motors
    m_indexerMotor.Configure(  m_indexerMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_indexerMotor.ClearFaults();
    
    // Set the motor configs
    m_clodeMotorConfig.SetIdleMode(IndexerConstants::clodeMotor::IDLE_MODE)
        .Inverted(IndexerConstants::clodeMotor::INVERTED)
        .SmartCurrentLimit(IndexerConstants::clodeMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(IndexerConstants::clodeMotor::RAMP_RATE)
        .VoltageCompensation(IndexerConstants::clodeMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_clodeMotor.Configure(  m_indexerMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_clodeMotor.ClearFaults();
}

void IndexerIOSpark::UpdateInputs(IndexerIOInputs& inputs) 
{
        inputs.isindexerMotorConnected = (m_indexerMotor.GetBusVoltage() !=0.0) && !m_indexerMotor.GetFaults().can;

    inputs.indexerMotorAppliedVoltage = m_indexerMotor.GetAppliedOutput() * IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION;
    inputs.indexerMotorBusVoltage = m_indexerMotor.GetBusVoltage();
    inputs.indexerMotorCurrent = m_indexerMotor.GetOutputCurrent();
    inputs.indexerMotorTemperature = m_indexerMotor.GetMotorTemperature();
    
    inputs.isClodeMotorConnected = (m_clodeMotor.GetBusVoltage() !=0.0) && !m_clodeMotor.GetFaults().can;

    inputs.clodeAppliedVoltage = m_clodeMotor.GetAppliedOutput() * IndexerConstants::clodeMotor::VOLTAGE_COMPENSATION;
    inputs.clodeBusVoltage = m_clodeMotor.GetBusVoltage();
    inputs.clodeCurrent = m_clodeMotor.GetOutputCurrent();
    inputs.clodeTemperature = m_clodeMotor.GetMotorTemperature();

    if(m_bestSensor.Get() == IndexerConstants::theMostImportantSensorOfTheRobot::IS_TRIGERED && !inputs.wasTriggered)
    {
        inputs.nbrOfBallShot++;
    }

    inputs.wasTriggered = m_bestSensor.Get();


    frc::SmartDashboard::PutNumber("indexer/IndexerVelocity", m_indexerMotor.GetEncoder().GetVelocity()/3);
}

void IndexerIOSpark::SetVoltage(double voltage, double clodeVoltage)
{
        DEBUG_ASSERT((voltage <= IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -IndexerConstants::indexerMotor::VOLTAGE_COMPENSATION) 
        ,"Indexer Voltage out of range");
    
    m_indexerMotor.SetVoltage(units::volt_t(voltage));
    m_clodeMotor.SetVoltage(units::volt_t(clodeVoltage));
}

void IndexerIOSpark::SetDutyCycle(double dutyCycle, double clodeDutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Indexer Duty Cycle out of range");
        m_indexerMotor.Set(dutyCycle);
        m_clodeMotor.Set(clodeDutyCycle);
}

void IndexerIOSpark::SetVelocity(double targetVelocity, double clodeVoltage)
{
    m_indexerPIDFController.SetSetpoint(targetVelocity,rev::spark::SparkLowLevel::ControlType::kVelocity);
}

