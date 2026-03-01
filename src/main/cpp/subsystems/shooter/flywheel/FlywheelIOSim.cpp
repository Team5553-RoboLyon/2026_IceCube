#include "subsystems/shooter/flywheel/FlywheelIOSim.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

FlywheelIOSim::FlywheelIOSim()
{

}

void FlywheelIOSim::UpdateInputs(FlywheelIOInputs& inputs) 
{
    m_flywheelSim.Update(0.02_s);
    inputs.isLeftMotorConnected = true;

    inputs.leftMotorAppliedVoltage = double(m_flywheelSim.GetInputVoltage());
    inputs.leftMotorBusVoltage = 12.0;
    inputs.leftMotorCurrent = m_flywheelSim.GetCurrentDraw().value();
    inputs.leftMotorTemperature = 23.0;
    inputs.leftMotorInternalEncoderVelocity = units::revolutions_per_minute_t(m_flywheelSim.GetAngularVelocity()).value();

    inputs.isRightMotorConnected = inputs.isLeftMotorConnected;

    inputs.rightMotorAppliedVoltage = inputs.leftMotorAppliedVoltage;
    inputs.rightMotorBusVoltage = inputs.leftMotorBusVoltage;
    inputs.rightMotorCurrent = inputs.leftMotorCurrent;
    inputs.rightMotorTemperature = inputs.leftMotorTemperature;
    inputs.rightMotorInternalEncoderVelocity = inputs.leftMotorInternalEncoderVelocity;

    inputs.shooterVelocity = (inputs.leftMotorInternalEncoderVelocity + inputs.rightMotorInternalEncoderVelocity) / 2; // Convert from RPS to RPM

    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/AppliedVoltage",inputs.leftMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/BusVoltage", inputs.leftMotorBusVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/Current", inputs.leftMotorCurrent);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/Temperature", inputs.leftMotorTemperature);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Left/InternalEncoderVelocity", inputs.leftMotorInternalEncoderVelocity);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/AppliedVoltage",inputs.rightMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/BusVoltage", inputs.rightMotorBusVoltage);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/Current", inputs.rightMotorCurrent);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/Temperature", inputs.rightMotorTemperature);
    frc::SmartDashboard::PutNumber("shooter/flywheel/Motor/Right/InternalEncoderVelocity", inputs.rightMotorInternalEncoderVelocity);
    frc::SmartDashboard::PutNumber("shooter/flywheel/FlywheelVelocity", inputs.shooterVelocity);

    frc::SmartDashboard::PutNumber("shooter/flywheel/simulator/AngularAcceleration", m_flywheelSim.GetAngularAcceleration().value());
    frc::SmartDashboard::PutNumber("shooter/flywheel/simulator/Torque", m_flywheelSim.GetTorque().value());
}

void FlywheelIOSim::SetVoltage(units::volt_t voltage)
{
    DEBUG_ASSERT((double(voltage) <= FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION) 
        && (double(voltage) >= -FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION) 
        ,"Flywheel voltage out of range");
    m_flywheelSim.SetInputVoltage(voltage);
}

void FlywheelIOSim::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Flywheel duty Cycle out of range");
    m_flywheelSim.SetInputVoltage(units::volt_t(dutyCycle * FlywheelConstants::RightMotor::VOLTAGE_COMPENSATION));
}

void FlywheelIOSim::SetVelocity(units::angular_velocity::revolutions_per_minute_t velocity)
{
    DEBUG_ASSERT((double(velocity) <= FlywheelConstants::Speed::MAX) 
        && (double(velocity) >= FlywheelConstants::Speed::MIN) 
        ,"Flywheel velocity out of range");

    DEBUG_ASSERT(false, "I think you're not patient enough to wait for the velocity control to be implemented. Don't worry, it'll be there soon :)");
}