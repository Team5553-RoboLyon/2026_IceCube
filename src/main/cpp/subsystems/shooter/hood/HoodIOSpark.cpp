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
    inputs.isMotorConnected = (m_hoodMotor.GetBusVoltage() !=0.0) && !m_hoodMotor.GetFaults().can;

    inputs.motorAppliedVoltage = m_hoodMotor.GetAppliedOutput() * HoodConstants::HoodMotor::VOLTAGE_COMPENSATION;
    inputs.motorBusVoltage = m_hoodMotor.GetBusVoltage();
    inputs.motorCurrent = m_hoodMotor.GetOutputCurrent();
    inputs.motorTemperature = m_hoodMotor.GetMotorTemperature();

    inputs.hoodAngle = m_hoodEncoder.GetDistance();

    #ifdef HOOD_SMARTDASHBOARD_LOG
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/AppliedVoltage",inputs.motorAppliedVoltage);
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/BusVoltage", inputs.motorBusVoltage);
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/Current", inputs.motorCurrent);
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/Temperature", inputs.motorTemperature);

    frc::SmartDashboard::PutNumber("shooter/hood/Position", inputs.hoodAngle);
    #else 
    m_logger.Log(inputs);
    #endif
}

void HoodIOSpark::SetVoltage(units::volt_t voltage)
{
    DEBUG_ASSERT((double(voltage) <= HoodConstants::HoodMotor::VOLTAGE_COMPENSATION) 
        && (double(voltage) >= -HoodConstants::HoodMotor::VOLTAGE_COMPENSATION) 
        ,"Hood voltage out of range");
    
    m_hoodMotor.SetVoltage(voltage);
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
