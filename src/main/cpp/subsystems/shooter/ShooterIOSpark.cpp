#include "subsystems/shooter/shooterIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

ShooterIOSpark::ShooterIOSpark()
{
        // Set the motor configs
    m_LeftMotorConfig.SetIdleMode(ShooterConstants::LeftMotor::IDLE_MODE)
        .Inverted(ShooterConstants::LeftMotor::INVERTED)
        .SmartCurrentLimit(ShooterConstants::LeftMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(ShooterConstants::LeftMotor::RAMP_RATE)
        .VoltageCompensation(ShooterConstants::LeftMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_LeftMotor.Configure(  m_LeftMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_LeftMotor.ClearFaults();
    // Set the motor configs
    m_RightMotorConfig.SetIdleMode(ShooterConstants::RightMotor::IDLE_MODE)
        .Inverted(ShooterConstants::RightMotor::INVERTED)
        .SmartCurrentLimit(ShooterConstants::RightMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(ShooterConstants::RightMotor::RAMP_RATE)
        .VoltageCompensation(ShooterConstants::RightMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_RightMotor.Configure(  m_RightMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_RightMotor.ClearFaults();
        m_WheelEncoder.Reset();
    m_WheelEncoder.SetDistancePerPulse(ShooterConstants::WheelEncoder::DISTANCE_PER_PULSE);

}

void ShooterIOSpark::UpdateInputs(ShooterIOInputs& inputs) 
{
    inputs.isLeftMotorConnected = (m_LeftMotor.GetBusVoltage() !=0.0) && !m_LeftMotor.GetFaults().can;

    inputs.LeftMotorAppliedVoltage = m_LeftMotor.GetAppliedOutput() * ShooterConstants::LeftMotor::VOLTAGE_COMPENSATION;
    inputs.LeftMotorBusVoltage = m_LeftMotor.GetBusVoltage();
    inputs.LeftMotorCurrent = m_LeftMotor.GetOutputCurrent();
    inputs.LeftMotorTemperature = m_LeftMotor.GetMotorTemperature();
    inputs.LeftMotorEncoderVelocity = m_LeftMotor.GetEncoder().GetVelocity();

    inputs.isRightMotorConnected = (m_RightMotor.GetBusVoltage() !=0.0) && !m_RightMotor.GetFaults().can;

    inputs.RightMotorAppliedVoltage = m_RightMotor.GetAppliedOutput() * ShooterConstants::RightMotor::VOLTAGE_COMPENSATION;
    inputs.RightMotorBusVoltage = m_RightMotor.GetBusVoltage();
    inputs.RightMotorCurrent = m_RightMotor.GetOutputCurrent();
    inputs.RightMotorTemperature = m_RightMotor.GetMotorTemperature();
    inputs.RightMotorEncoderVelocity = m_RightMotor.GetEncoder().GetVelocity();

    inputs.ShooterVelocity = m_WheelEncoder.GetRate() * 60.0; // Convert from RPS to RPM
    inputs.rotation = m_WheelEncoder.GetDistance();
    inputs.FuelLaunched = m_IRBreakerOutput.Get() == ShooterConstants::IRBreakerOutput::IS_TRIGGERED;
}

void ShooterIOSpark::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= ShooterConstants::LeftMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -ShooterConstants::LeftMotor::VOLTAGE_COMPENSATION) 
        ,"Shooter Voltage out of range");
    
    m_LeftMotor.SetVoltage(units::volt_t(voltage));
    DEBUG_ASSERT((voltage <= ShooterConstants::RightMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -ShooterConstants::RightMotor::VOLTAGE_COMPENSATION) 
        ,"Shooter Voltage out of range");
    
    m_RightMotor.SetVoltage(units::volt_t(voltage));
}

void ShooterIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Shooter Duty Cycle out of range");
        m_LeftMotor.Set(dutyCycle);
    m_RightMotor.Set(dutyCycle);
}

void ShooterIOSpark::ResetRotation() 
{
    m_WheelEncoder.Reset();
}
