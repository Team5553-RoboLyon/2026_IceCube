#include "subsystems/shooter/flywheel/FlywheelLogger.h"

FlywheelIOLogger::FlywheelIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isLeftMotorConnected(log, path + "/LeftMotor" + "/IO/isConnected"),
          leftMotorCurrent(log, path + "/LeftMotor" + "/Current"),
          leftMotorAppliedVoltage(log, path + "/LeftMotor" + "/AppliedVoltage"),
          leftMotorBusVoltage(log, path + "/LeftMotor" + "/BusVoltage"),
          leftMotorTemperature(log, path + "/LeftMotor" + "/Temperature"),
            leftMotorInternalEncoderVelocity(log, path + "/LeftMotor" + "/InternalEncoderVelocity"),
            isRightMotorConnected(log, path + "/RightMotor" + "/IO/isConnected"),
            rightMotorCurrent(log, path + "/RightMotor" + "/Current"),
            rightMotorAppliedVoltage(log, path + "/RightMotor" + "/AppliedVoltage"),
            rightMotorBusVoltage(log, path + "/RightMotor" + "/BusVoltage"),
            rightMotorTemperature(log, path + "/RightMotor" + "/Temperature"),
            rightMotorInternalEncoderVelocity(log, path + "/RightMotor" + "/InternalEncoderVelocity"),
            shooterVelocity(log, path + "/ShooterVelocity"),
          
          m_path(path)
{}

void FlywheelIOLogger::Log(const FlywheelIOInputs& inputs) {
    isLeftMotorConnected.Append(inputs.isLeftMotorConnected);
    leftMotorCurrent.Append(inputs.leftMotorCurrent);
    leftMotorAppliedVoltage.Append(inputs.leftMotorAppliedVoltage);
    leftMotorBusVoltage.Append(inputs.leftMotorBusVoltage);
    leftMotorTemperature.Append(inputs.leftMotorTemperature);
    leftMotorInternalEncoderVelocity.Append(inputs.leftMotorInternalEncoderVelocity);
    isRightMotorConnected.Append(inputs.isRightMotorConnected);
    rightMotorCurrent.Append(inputs.rightMotorCurrent);
    rightMotorAppliedVoltage.Append(inputs.rightMotorAppliedVoltage);
    rightMotorBusVoltage.Append(inputs.rightMotorBusVoltage);
    rightMotorTemperature.Append(inputs.rightMotorTemperature);
    rightMotorInternalEncoderVelocity.Append(inputs.rightMotorInternalEncoderVelocity);
    shooterVelocity.Append(inputs.shooterVelocity);
}