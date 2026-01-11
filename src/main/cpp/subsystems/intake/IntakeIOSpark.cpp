#include "subsystems/intake/intakeIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

IntakeIOSpark::IntakeIOSpark()
{
        // Set the motor configs
    m_leftMotorConfig.SetIdleMode(IntakeConstants::leftMotor::IDLE_MODE)
        .Inverted(IntakeConstants::leftMotor::INVERTED)
        .SmartCurrentLimit(IntakeConstants::leftMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(IntakeConstants::leftMotor::RAMP_RATE)
        .VoltageCompensation(IntakeConstants::leftMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_leftMotor.Configure(  m_leftMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_leftMotor.ClearFaults();
    // Set the motor configs
    m_rightMotorConfig.SetIdleMode(IntakeConstants::rightMotor::IDLE_MODE)
        .Inverted(IntakeConstants::rightMotor::INVERTED)
        .SmartCurrentLimit(IntakeConstants::rightMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(IntakeConstants::rightMotor::RAMP_RATE)
        .VoltageCompensation(IntakeConstants::rightMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_rightMotor.Configure(  m_rightMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_rightMotor.ClearFaults();
    
}

void IntakeIOSpark::UpdateInputs(IntakeIOInputs& inputs) 
{
        inputs.isleftMotorConnected = (m_leftMotor.GetBusVoltage() !=0.0) && !m_leftMotor.GetFaults().can;

    inputs.leftMotorAppliedVoltage = m_leftMotor.GetAppliedOutput() * IntakeConstants::leftMotor::VOLTAGE_COMPENSATION;
    inputs.leftMotorBusVoltage = m_leftMotor.GetBusVoltage();
    inputs.leftMotorCurrent = m_leftMotor.GetOutputCurrent();
    inputs.leftMotorTemperature = m_leftMotor.GetMotorTemperature();
    
    inputs.isrightMotorConnected = (m_rightMotor.GetBusVoltage() !=0.0) && !m_rightMotor.GetFaults().can;

    inputs.rightMotorAppliedVoltage = m_rightMotor.GetAppliedOutput() * IntakeConstants::rightMotor::VOLTAGE_COMPENSATION;
    inputs.rightMotorBusVoltage = m_rightMotor.GetBusVoltage();
    inputs.rightMotorCurrent = m_rightMotor.GetOutputCurrent();
    inputs.rightMotorTemperature = m_rightMotor.GetMotorTemperature();
    
    
}

void IntakeIOSpark::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= IntakeConstants::leftMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -IntakeConstants::leftMotor::VOLTAGE_COMPENSATION) 
        ,"Intake Voltage out of range");
    
    m_leftMotor.SetVoltage(units::volt_t(voltage));
    DEBUG_ASSERT((voltage <= IntakeConstants::rightMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -IntakeConstants::rightMotor::VOLTAGE_COMPENSATION) 
        ,"Intake Voltage out of range");
    
    m_rightMotor.SetVoltage(units::volt_t(voltage));
}

void IntakeIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Intake Duty Cycle out of range");
        m_leftMotor.Set(dutyCycle);
    m_rightMotor.Set(dutyCycle);
}

