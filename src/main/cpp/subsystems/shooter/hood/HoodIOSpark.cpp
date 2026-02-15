#include "subsystems/shooter/hood/HoodIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

HoodIOSpark::HoodIOSpark()
{
        // Set the motor configs
    m_hoodMotorConfig.SetIdleMode(HoodConstants::HoodMotor::IDLE_MODE)
        .Inverted(HoodConstants::HoodMotor::INVERTED)
        .SmartCurrentLimit(HoodConstants::HoodMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(HoodConstants::HoodMotor::RAMP_RATE)
        .VoltageCompensation(HoodConstants::HoodMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_hoodMotor.Configure(  m_hoodMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_hoodMotor.ClearFaults();
    
    m_hoodEncoder.SetDistancePerPulse(HoodConstants::HoodEncoder::DISTANCE_PER_PULSE);
    m_hoodEncoder.Reset();
}

void HoodIOSpark::UpdateInputs(HoodIOInputs& inputs) 
{
    inputs.isHoodMotorConnected = (m_hoodMotor.GetBusVoltage() !=0.0) && !m_hoodMotor.GetFaults().can;

    inputs.hoodMotorAppliedVoltage = m_hoodMotor.GetAppliedOutput() * HoodConstants::HoodMotor::VOLTAGE_COMPENSATION;
    inputs.hoodMotorBusVoltage = m_hoodMotor.GetBusVoltage();
    inputs.hoodMotorCurrent = m_hoodMotor.GetOutputCurrent();
    inputs.hoodMotorTemperature = m_hoodMotor.GetMotorTemperature();

    inputs.hoodPos = m_hoodEncoder.GetDistance();

    frc::SmartDashboard::PutNumber("HoodAppliedVoltage",inputs.hoodMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("HoodVoltage", inputs.hoodMotorBusVoltage);
    frc::SmartDashboard::PutNumber("HoodCurrent", inputs.hoodMotorCurrent);
    frc::SmartDashboard::PutNumber("HoodTemperature", inputs.hoodMotorTemperature);

    frc::SmartDashboard::PutNumber("HoodPos", inputs.hoodPos);
}

void HoodIOSpark::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= HoodConstants::HoodMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -HoodConstants::HoodMotor::VOLTAGE_COMPENSATION) 
        ,"Hood bottomVoltage out of range");
    
    m_hoodMotor.SetVoltage(units::volt_t(voltage));
}

void HoodIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Hood Duty Cycle out of range");
    m_hoodMotor.Set(dutyCycle);
}

void HoodIOSpark::ResetEncoder()
{
    m_hoodEncoder.Reset();
}
