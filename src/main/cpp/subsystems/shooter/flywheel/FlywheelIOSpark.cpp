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

    inputs.leftMotorBusVoltage = m_leftMotor.GetBusVoltage();
    inputs.leftMotorAppliedVoltage = m_leftMotor.GetAppliedOutput() * inputs.leftMotorBusVoltage;
    inputs.leftMotorCurrent = m_leftMotor.GetOutputCurrent();
    inputs.leftMotorTemperature = m_leftMotor.GetMotorTemperature();
    inputs.leftMotorInternalEncoderVelocity = m_leftMotor.GetEncoder().GetVelocity();

    inputs.isRightMotorConnected = (m_rightMotor.GetBusVoltage() !=0.0) && !m_rightMotor.GetFaults().can;

    inputs.rightMotorBusVoltage = m_rightMotor.GetBusVoltage();
    inputs.rightMotorAppliedVoltage = m_rightMotor.GetAppliedOutput() * inputs.rightMotorBusVoltage;
    inputs.rightMotorCurrent = m_rightMotor.GetOutputCurrent();
    inputs.rightMotorTemperature = m_rightMotor.GetMotorTemperature();
    inputs.rightMotorInternalEncoderVelocity = m_rightMotor.GetEncoder().GetVelocity();

    inputs.shooterVelocity = (inputs.leftMotorInternalEncoderVelocity + inputs.rightMotorInternalEncoderVelocity) / 2; // Convert from RPS to RPM

    #ifdef FLYWHEEL_SMARTDASHBOARD_LOG
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/AppliedVoltage",inputs.leftMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/BusVoltage", inputs.leftMotorBusVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/Current", inputs.leftMotorCurrent);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/Temperature", inputs.leftMotorTemperature);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/AppliedVoltage",inputs.rightMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/BusVoltage", inputs.rightMotorBusVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/Current", inputs.rightMotorCurrent);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/Temperature", inputs.rightMotorTemperature);

    frc::SmartDashboard::PutNumber("shooter/flywheel/FlywheelVelocity", inputs.shooterVelocity);
    #else
    m_logger.Log(inputs);
    #endif
}

void FlywheelIOSpark::SetVoltage(units::volt_t voltage)
{
    DEBUG_ASSERT((double(voltage) <= FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION) 
        && (double(voltage) >= -FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION) 
        ,"Flywheel voltage out of range");
    
    m_leftMotor.SetVoltage(voltage);
    m_rightMotor.SetVoltage(voltage);
}

void FlywheelIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Flywheel duty Cycle out of range");
    m_leftMotor.Set(dutyCycle);
    m_rightMotor.Set(dutyCycle);
}

void FlywheelIOSpark::SetVelocity(units::angular_velocity::revolutions_per_minute_t velocity)
{
    DEBUG_ASSERT((double(velocity) <= FlywheelConstants::Speed::MAX) 
        && (double(velocity) >= FlywheelConstants::Speed::MIN) 
        ,"Flywheel velocity out of range");

    // m_leftClosedLoopController.SetSetpoint(double(velocity), rev::spark::SparkLowLevel::ControlType::kVelocity);
    // m_rightClosedLoopController.SetSetpoint(double(velocity), rev::spark::SparkLowLevel::ControlType::kVelocity);
}