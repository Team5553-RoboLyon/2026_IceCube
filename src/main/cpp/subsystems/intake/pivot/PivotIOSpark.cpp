#include "subsystems/intake/pivot/PivotIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

PivotIOSpark::PivotIOSpark()
{
    // Set the motor configs
    m_pivotMotorConfig.SetIdleMode(PivotConstants::pivotMotor::IDLE_MODE)
                      .Inverted(PivotConstants::pivotMotor::INVERTED)
                      .SmartCurrentLimit(PivotConstants::pivotMotor::CURRENT_LIMIT)
                      .ClosedLoopRampRate(PivotConstants::pivotMotor::RAMP_RATE)
                      .VoltageCompensation(PivotConstants::pivotMotor::VOLTAGE_COMPENSATION);

    // Apply the configs to the motors
    m_pivotMotor.Configure(  m_pivotMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_pivotMotor.ClearFaults();

    m_rightEncoder.SetInverted(PivotConstants::EncoderRight::INVERTED);
    m_leftEncoder.SetInverted(PivotConstants::EncoderLeft::INVERTED);
}

void PivotIOSpark::UpdateInputs(PivotIOInputs& inputs) 
{    
    inputs.isPivotMotorConnected = (m_pivotMotor.GetBusVoltage() !=0.0) && !m_pivotMotor.GetFaults().can;

    inputs.pivotMotorAppliedVoltage = m_pivotMotor.GetAppliedOutput() * PivotConstants::pivotMotor::VOLTAGE_COMPENSATION;
    inputs.pivotMotorBusVoltage = m_pivotMotor.GetBusVoltage();
    inputs.pivotMotorCurrent = m_pivotMotor.GetOutputCurrent();
    inputs.pivotMotorTemperature = m_pivotMotor.GetMotorTemperature();

    inputs.isLeftEncoderConnected = m_leftEncoder.IsConnected();
    inputs.isRightEncoderConnected = m_rightEncoder.IsConnected();

    if (inputs.isLeftEncoderConnected && inputs.isLeftEncoderConnected)
    {
        inputs.pivotPos = (m_leftEncoder.Get()+m_rightEncoder.Get())/2;
    }
    else if (!inputs.isLeftEncoderConnected && inputs.isLeftEncoderConnected)
    {
        inputs.pivotPos = m_rightEncoder.Get();
    }
    else if (inputs.isLeftEncoderConnected && !inputs.isLeftEncoderConnected)
    {
        inputs.pivotPos = m_leftEncoder.Get();
    }

    frc::SmartDashboard::PutNumber("intake/PivotPos", inputs.pivotPos);
    frc::SmartDashboard::PutNumber("intake/PivotTargetPos", m_pivotMotorController.GetSetpoint());
}

void PivotIOSpark::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= PivotConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -PivotConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        ,"Pivot Voltage out of range");

    m_pivotMotor.SetVoltage(units::volt_t(voltage));
}

void PivotIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Intake Duty Cycle out of range");
    m_pivotMotor.Set(dutyCycle);    
}