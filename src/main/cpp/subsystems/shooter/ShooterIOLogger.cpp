#include "subsystems/shooter/shooterIOLogger.h"

ShooterIOLogger::ShooterIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isLeftMotorConnected(log, path + "/Flywheel/LeftMotor" + "/isConnected"),
          LeftMotorCurrent(log, path + "/Flywheel/LeftMotor" + "/Current"),
          LeftMotorAppliedVoltage(log, path + "/Flywheel/LeftMotor" + "/AppliedVoltage"),
          LeftMotorBusVoltage(log, path + "/Flywheel/LeftMotor" + "/BusVoltage"),
          LeftMotorTemperature(log, path + "/Flywheel/LeftMotor" + "/Temperature"),
          LeftMotorEncoderVelocity(log, path + "/Flywheel/LeftMotor" + "/EncoderVelocity"),
          isRightMotorConnected(log, path + "/Flywheel/RightMotor" + "/isConnected"),
          RightMotorCurrent(log, path + "/Flywheel/RightMotor" + "/Current"),
          RightMotorAppliedVoltage(log, path + "/Flywheel/RightMotor" + "/AppliedVoltage"),
          RightMotorBusVoltage(log, path + "/Flywheel/RightMotor" + "/BusVoltage"),
          RightMotorTemperature(log, path + "/Flywheel/RightMotor" + "/Temperature"),
          RightMotorEncoderVelocity(log, path + "/Flywheel/RightMotor" + "/EncoderVelocity"),
          ShooterVelocity(log, path + "/Flywheel" + "/FlywheelVelocity"),
          isHoodMotorConnected(log, path + "/Hood/HoodMotor" + "/isConnected"),
          HoodMotorCurrent(log, path + "/Hood/HoodMotor" + "/Current"),
          HoodMotorAppliedVoltage(log, path + "/Hood/HoodMotor" + "/AppliedVoltage"),
          HoodMotorBusVoltage(log, path + "/Hood/HoodMotor" + "/BusVoltage"),
          HoodMotorTemperature(log, path + "/Hood/HoodMotor" + "/Temperature"),
          HoodPos(log, path + "/Hood" + "/HoodPos"),
          m_path(path)
{}

void ShooterIOLogger::Log(const HoodIOInputs& hoodInputs, const FlywheelIOInputs& flywheelInputs) {
        isLeftMotorConnected.Append(flywheelInputs.isLeftMotorConnected);
    LeftMotorAppliedVoltage.Append(flywheelInputs.leftMotorAppliedVoltage);
    LeftMotorBusVoltage.Append(flywheelInputs.leftMotorBusVoltage);
    LeftMotorCurrent.Append(flywheelInputs.leftMotorCurrent);
    LeftMotorTemperature.Append(flywheelInputs.leftMotorTemperature);
    LeftMotorEncoderVelocity.Append(flywheelInputs.leftMotorEncoderVelocity);
    isRightMotorConnected.Append(flywheelInputs.isRightMotorConnected);
    RightMotorAppliedVoltage.Append(flywheelInputs.rightMotorAppliedVoltage);
    RightMotorBusVoltage.Append(flywheelInputs.rightMotorBusVoltage);
    RightMotorCurrent.Append(flywheelInputs.rightMotorCurrent);
    RightMotorTemperature.Append(flywheelInputs.rightMotorTemperature);
    RightMotorEncoderVelocity.Append(flywheelInputs.rightMotorEncoderVelocity);
    ShooterVelocity.Append(flywheelInputs.ShooterVelocity);
    isHoodMotorConnected.Append(hoodInputs.isHoodMotorConnected);
    HoodMotorCurrent.Append(hoodInputs.hoodMotorCurrent);
    HoodMotorAppliedVoltage.Append(hoodInputs.hoodMotorAppliedVoltage);
    HoodMotorBusVoltage.Append(hoodInputs.hoodMotorBusVoltage);
    HoodMotorTemperature.Append(hoodInputs.hoodMotorTemperature);
    HoodPos.Append(hoodInputs.hoodPos);
}