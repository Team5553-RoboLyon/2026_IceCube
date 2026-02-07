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
    // Set the motor configs
    m_bottomMotorConfig.SetIdleMode(ShooterConstants::BottomMotor::IDLE_MODE)
        .Inverted(ShooterConstants::BottomMotor::INVERTED)
        .SmartCurrentLimit(ShooterConstants::BottomMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(ShooterConstants::BottomMotor::RAMP_RATE)
        .VoltageCompensation(ShooterConstants::BottomMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_bottomMotor.Configure(  m_bottomMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_bottomMotor.ClearFaults();
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

    inputs.isBottomMotorConnected = (m_bottomMotor.GetBusVoltage() !=0.0) && !m_bottomMotor.GetFaults().can;

    inputs.BottomMotorAppliedVoltage = m_bottomMotor.GetAppliedOutput() * ShooterConstants::BottomMotor::VOLTAGE_COMPENSATION;
    inputs.BottomMotorBusVoltage = m_bottomMotor.GetBusVoltage();
    inputs.BottomMotorCurrent = m_bottomMotor.GetOutputCurrent();
    inputs.BottomMotorTemperature = m_bottomMotor.GetMotorTemperature();
    inputs.BottomMotorEncoderVelocity = m_bottomMotor.GetEncoder().GetVelocity();

    frc::SmartDashboard::PutNumber("leftV",inputs.LeftMotorEncoderVelocity);
    frc::SmartDashboard::PutNumber("voltageL", inputs.LeftMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("CurrentL", inputs.LeftMotorCurrent);
    frc::SmartDashboard::PutNumber("temperatureL", inputs.LeftMotorTemperature);
    frc::SmartDashboard::PutNumber("rightV",inputs.RightMotorEncoderVelocity);
    frc::SmartDashboard::PutNumber("voltageR", inputs.RightMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("CurrentR", inputs.RightMotorCurrent);
    frc::SmartDashboard::PutNumber("temperatureR", inputs.RightMotorTemperature);
    frc::SmartDashboard::PutNumber("bottomV",inputs.BottomMotorEncoderVelocity);
    frc::SmartDashboard::PutNumber("voltageB", inputs.BottomMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("CurrentB", inputs.BottomMotorCurrent);
    frc::SmartDashboard::PutNumber("temperatureB", inputs.BottomMotorTemperature);

    frc::SmartDashboard::PutNumber("Rotation", inputs.rotation);
}

void ShooterIOSpark::SetVoltage(double upVoltage, double bottomVoltage)
{
        DEBUG_ASSERT((upVoltage <= ShooterConstants::LeftMotor::VOLTAGE_COMPENSATION) 
        && (upVoltage >= -ShooterConstants::LeftMotor::VOLTAGE_COMPENSATION) 
        ,"Shooter upVoltage out of range");
    
    m_LeftMotor.SetVoltage(units::volt_t(upVoltage));
    DEBUG_ASSERT((upVoltage <= ShooterConstants::RightMotor::VOLTAGE_COMPENSATION) 
        && (upVoltage >= -ShooterConstants::RightMotor::VOLTAGE_COMPENSATION) 
        ,"Shooter upVoltage out of range");
    
    m_RightMotor.SetVoltage(units::volt_t(upVoltage));

    DEBUG_ASSERT((bottomVoltage <= ShooterConstants::BottomMotor::VOLTAGE_COMPENSATION) 
        && (bottomVoltage >= -ShooterConstants::BottomMotor::VOLTAGE_COMPENSATION) 
        ,"Shooter bottomVoltage out of range");
    
    m_bottomMotor.SetVoltage(units::volt_t(bottomVoltage));
}

void ShooterIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Shooter Duty Cycle out of range");
        m_LeftMotor.Set(dutyCycle);
    m_RightMotor.Set(dutyCycle);
    m_bottomMotor.Set(dutyCycle);
}

void ShooterIOSpark::ResetRotation() 
{
    m_WheelEncoder.Reset();
}
