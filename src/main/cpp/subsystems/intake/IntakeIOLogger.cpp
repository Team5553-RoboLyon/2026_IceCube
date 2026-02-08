#include "subsystems/intake/intakeIOLogger.h"

IntakeIOLogger::IntakeIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isintakeMotorConnected(log, path + "/intakeMotor" + "/isConnected"),
          intakeMotorCurrent(log, path + "/intakeMotor" + "/Current"),
          intakeMotorAppliedVoltage(log, path + "/intakeMotor" + "/AppliedVoltage"),
          intakeMotorBusVoltage(log, path + "/intakeMotor" + "/BusVoltage"),
          intakeMotorTemperature(log, path + "/intakeMotor" + "/Temperature"),
          ispivotMotorConnected(log, path + "/pivotMotor" + "/isConnected"),
          pivotMotorCurrent(log, path + "/pivotMotor" + "/Current"),
          pivotMotorAppliedVoltage(log, path + "/pivotMotor" + "/AppliedVoltage"),
          pivotMotorBusVoltage(log, path + "/pivotMotor" + "/BusVoltage"),
          pivotMotorTemperature(log, path + "/pivotMotor" + "/Temperature"),
          isMichelMotorConnected(log, path + "/pivotMotor" + "/isConnected"),
          michelMotorCurrent(log, path + "/pivotMotor" + "/Current"),
          michelMotorAppliedVoltage(log, path + "/michelMotor" + "/AppliedVoltage"),
          michelMotorBusVoltage(log, path + "/michelMotor" + "/BusVoltage"),
          michelMotorTemperature(log, path + "/michelMotor" + "/Temperature"),
          
          m_path(path)
{}

void IntakeIOLogger::Log(const IntakeIOInputs& inputs) {
        isintakeMotorConnected.Append(inputs.isIntakeMotorConnected);
    intakeMotorAppliedVoltage.Append(inputs.intakeMotorAppliedVoltage);
    intakeMotorBusVoltage.Append(inputs.intakeMotorBusVoltage);
    intakeMotorCurrent.Append(inputs.intakeMotorCurrent);
    intakeMotorTemperature.Append(inputs.intakeMotorTemperature);
    ispivotMotorConnected.Append(inputs.isPivotMotorConnected);
    pivotMotorAppliedVoltage.Append(inputs.pivotMotorAppliedVoltage);
    pivotMotorBusVoltage.Append(inputs.pivotMotorBusVoltage);
    pivotMotorCurrent.Append(inputs.pivotMotorCurrent);
    pivotMotorTemperature.Append(inputs.pivotMotorTemperature);
    isMichelMotorConnected.Append(inputs.isMichelMotorConnected);
    michelMotorAppliedVoltage.Append(inputs.pivotMotorAppliedVoltage);
    michelMotorBusVoltage.Append(inputs.pivotMotorBusVoltage);
    michelMotorCurrent.Append(inputs.pivotMotorCurrent);
    michelMotorTemperature.Append(inputs.pivotMotorTemperature);
}