#include "subsystems/intake/intakeIOLogger.h"

IntakeIOLogger::IntakeIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isleftMotorConnected(log, path + "/leftMotor" + "/isConnected"),
          leftMotorCurrent(log, path + "/leftMotor" + "/Current"),
          leftMotorAppliedVoltage(log, path + "/leftMotor" + "/AppliedVoltage"),
          leftMotorBusVoltage(log, path + "/leftMotor" + "/BusVoltage"),
          leftMotorTemperature(log, path + "/leftMotor" + "/Temperature"),
          isrightMotorConnected(log, path + "/rightMotor" + "/isConnected"),
          rightMotorCurrent(log, path + "/rightMotor" + "/Current"),
          rightMotorAppliedVoltage(log, path + "/rightMotor" + "/AppliedVoltage"),
          rightMotorBusVoltage(log, path + "/rightMotor" + "/BusVoltage"),
          rightMotorTemperature(log, path + "/rightMotor" + "/Temperature"),
          
          m_path(path)
{}

void IntakeIOLogger::Log(const IntakeIOInputs& inputs) {
        isleftMotorConnected.Append(inputs.isleftMotorConnected);
    leftMotorAppliedVoltage.Append(inputs.leftMotorAppliedVoltage);
    leftMotorBusVoltage.Append(inputs.leftMotorBusVoltage);
    leftMotorCurrent.Append(inputs.leftMotorCurrent);
    leftMotorTemperature.Append(inputs.leftMotorTemperature);
    isrightMotorConnected.Append(inputs.isrightMotorConnected);
    rightMotorAppliedVoltage.Append(inputs.rightMotorAppliedVoltage);
    rightMotorBusVoltage.Append(inputs.rightMotorBusVoltage);
    rightMotorCurrent.Append(inputs.rightMotorCurrent);
    rightMotorTemperature.Append(inputs.rightMotorTemperature);
    
}