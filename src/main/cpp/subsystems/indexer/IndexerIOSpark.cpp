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

    inputs.indexerMotorBusVoltage = m_indexerMotor.GetBusVoltage();
    inputs.indexerMotorAppliedVoltage = m_indexerMotor.GetAppliedOutput() * inputs.indexerMotorBusVoltage;
    inputs.indexerMotorCurrent = m_indexerMotor.GetOutputCurrent();
    inputs.indexerMotorTemperature = m_indexerMotor.GetMotorTemperature();
    
    inputs.isClodeMotorConnected = (m_clodeMotor.GetBusVoltage() !=0.0) && !m_clodeMotor.GetFaults().can;

    inputs.clodeBusVoltage = m_clodeMotor.GetBusVoltage();
    inputs.clodeAppliedVoltage = m_clodeMotor.GetAppliedOutput() * inputs.clodeBusVoltage;
    inputs.clodeCurrent = m_clodeMotor.GetOutputCurrent();
    inputs.clodeTemperature = m_clodeMotor.GetMotorTemperature();

    inputs.wasThereABall = inputs.isThereABall;
    inputs.isThereABall = m_bestSensor.Get() == IndexerConstants::theMostImportantSensorOfTheRobot::IS_TRIGERED;
    inputs.nbrOfBallShot += inputs.wasThereABall && !inputs.isThereABall;

    frc::SmartDashboard::PutBoolean("indexer/feeder/Motor/Connected", inputs.isindexerMotorConnected);
    frc::SmartDashboard::PutNumber("indexer/feeder/Motor/AppliedVoltage",inputs.indexerMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("indexer/feeder/Motor/BusVoltage", inputs.indexerMotorBusVoltage);
    frc::SmartDashboard::PutNumber("indexer/feeder/Motor/Current", inputs.indexerMotorCurrent);
    frc::SmartDashboard::PutNumber("indexer/feeder/Motor/Temperature", inputs.indexerMotorTemperature);
    frc::SmartDashboard::PutBoolean("indexer/clode/Motor/Connected", inputs.isClodeMotorConnected);
    frc::SmartDashboard::PutNumber("indexer/clode/Motor/AppliedVoltage",inputs.clodeAppliedVoltage);
    frc::SmartDashboard::PutNumber("indexer/clode/Motor/BusVoltage", inputs.clodeBusVoltage);
    frc::SmartDashboard::PutNumber("indexer/clode/Motor/Current", inputs.clodeCurrent);
    frc::SmartDashboard::PutNumber("indexer/clode/Motor/Temperature", inputs.clodeTemperature);
    frc::SmartDashboard::PutNumber("indexer/Sensor/NumberOfBalls", inputs.nbrOfBallShot);
    frc::SmartDashboard::PutBoolean("indexer/Sensor/IsThereABall", inputs.isThereABall);
    frc::SmartDashboard::PutBoolean("indexer/Sensor/WasThereABall", inputs.wasThereABall);
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


