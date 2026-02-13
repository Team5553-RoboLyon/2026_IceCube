#include "subsystems/shooter/flywheel/FlywheelIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

FlywheelIOSpark::FlywheelIOSpark()
{
        // Set the motor configs
    m_leftMotorConfig.SetIdleMode(FlywheelConstants::LeftMotor::IDLE_MODE)
        .Inverted(FlywheelConstants::LeftMotor::INVERTED)
        .SmartCurrentLimit(FlywheelConstants::LeftMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(FlywheelConstants::LeftMotor::RAMP_RATE)
        .VoltageCompensation(FlywheelConstants::LeftMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_leftMotor.Configure(  m_leftMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_leftMotor.ClearFaults();
    // Set the motor configs
    m_rightMotorConfig.SetIdleMode(FlywheelConstants::RightMotor::IDLE_MODE)
        .Inverted(FlywheelConstants::RightMotor::INVERTED)
        .SmartCurrentLimit(FlywheelConstants::RightMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(FlywheelConstants::RightMotor::RAMP_RATE)
        .VoltageCompensation(FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_rightMotor.Configure(  m_rightMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_rightMotor.ClearFaults();
}

void FlywheelIOSpark::UpdateInputs(FlywheelIOInputs& inputs) 
{
    inputs.isLeftMotorConnected = (m_leftMotor.GetBusVoltage() !=0.0) && !m_leftMotor.GetFaults().can;

    inputs.leftMotorAppliedVoltage = m_leftMotor.GetAppliedOutput() * FlywheelConstants::LeftMotor::VOLTAGE_COMPENSATION;
    inputs.leftMotorBusVoltage = m_leftMotor.GetBusVoltage();
    inputs.leftMotorCurrent = m_leftMotor.GetOutputCurrent();
    inputs.leftMotorTemperature = m_leftMotor.GetMotorTemperature();
    inputs.leftMotorEncoderVelocity = m_leftMotor.GetEncoder().GetVelocity();

    inputs.isRightMotorConnected = (m_rightMotor.GetBusVoltage() !=0.0) && !m_rightMotor.GetFaults().can;

    inputs.rightMotorAppliedVoltage = m_rightMotor.GetAppliedOutput() * FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION;
    inputs.rightMotorBusVoltage = m_rightMotor.GetBusVoltage();
    inputs.rightMotorCurrent = m_rightMotor.GetOutputCurrent();
    inputs.rightMotorTemperature = m_rightMotor.GetMotorTemperature();
    inputs.rightMotorEncoderVelocity = m_rightMotor.GetEncoder().GetVelocity();

    inputs.ShooterVelocity = (inputs.leftMotorEncoderVelocity-inputs.rightMotorEncoderVelocity)/2; // Convert from RPS to RPM

    frc::SmartDashboard::PutNumber("LeftFlywheelAppliedVoltage",inputs.leftMotorEncoderVelocity);
    frc::SmartDashboard::PutNumber("LeftFlywheelVoltage", inputs.leftMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("LeftFlywheelCurrent", inputs.leftMotorCurrent);
    frc::SmartDashboard::PutNumber("LeftFlywheelTemperature", inputs.leftMotorTemperature);
    frc::SmartDashboard::PutNumber("RightFlywheelAppliedVoltage",inputs.rightMotorEncoderVelocity);
    frc::SmartDashboard::PutNumber("RightFlywheelVoltage", inputs.rightMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("RightFlywheelCurrent", inputs.rightMotorCurrent);
    frc::SmartDashboard::PutNumber("RightFlywheelTemperature", inputs.rightMotorTemperature);

    frc::SmartDashboard::PutNumber("FlywheelSpeed", inputs.ShooterVelocity);
}

void FlywheelIOSpark::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION) 
        ,"Flywheel voltage out of range");
    
    m_leftMotor.SetVoltage(units::volt_t(voltage));
    m_rightMotor.SetVoltage(units::volt_t(voltage));
}

void FlywheelIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Flywheel duty Cycle out of range");
    m_leftMotor.Set(dutyCycle);
    m_rightMotor.Set(dutyCycle);
}
