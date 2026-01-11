#include "subsystems/shooter/shooterIOLogger.h"

ShooterIOLogger::ShooterIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isLeftMotorConnected(log, path + "/LeftMotor" + "/isConnected"),
          LeftMotorCurrent(log, path + "/LeftMotor" + "/Current"),
          LeftMotorAppliedVoltage(log, path + "/LeftMotor" + "/AppliedVoltage"),
          LeftMotorBusVoltage(log, path + "/LeftMotor" + "/BusVoltage"),
          LeftMotorTemperature(log, path + "/LeftMotor" + "/Temperature"),
          LeftMotorEncoderVelocity(log, path + "/LeftMotor" + "/EncoderVelocity"),
          isRightMotorConnected(log, path + "/RightMotor" + "/isConnected"),
          RightMotorCurrent(log, path + "/RightMotor" + "/Current"),
          RightMotorAppliedVoltage(log, path + "/RightMotor" + "/AppliedVoltage"),
          RightMotorBusVoltage(log, path + "/RightMotor" + "/BusVoltage"),
          RightMotorTemperature(log, path + "/RightMotor" + "/Temperature"),
            RightMotorEncoderVelocity(log, path + "/RightMotor" + "/EncoderVelocity"),
                  rotation(log, path + "/WheelEncoder" + "/rotation"),
        FuelLaunched(log, path + "/IRBreakerOutput" + "/FuelLaunched"),
          m_path(path)
{}

void ShooterIOLogger::Log(const ShooterIOInputs& inputs) {
        isLeftMotorConnected.Append(inputs.isLeftMotorConnected);
    LeftMotorAppliedVoltage.Append(inputs.LeftMotorAppliedVoltage);
    LeftMotorBusVoltage.Append(inputs.LeftMotorBusVoltage);
    LeftMotorCurrent.Append(inputs.LeftMotorCurrent);
    LeftMotorTemperature.Append(inputs.LeftMotorTemperature);
    isRightMotorConnected.Append(inputs.isRightMotorConnected);
    RightMotorAppliedVoltage.Append(inputs.RightMotorAppliedVoltage);
    RightMotorBusVoltage.Append(inputs.RightMotorBusVoltage);
    RightMotorCurrent.Append(inputs.RightMotorCurrent);
    RightMotorTemperature.Append(inputs.RightMotorTemperature);
    LeftMotorEncoderVelocity.Append(inputs.LeftMotorEncoderVelocity);
    RightMotorEncoderVelocity.Append(inputs.RightMotorEncoderVelocity);
        rotation.Append(inputs.rotation);

}