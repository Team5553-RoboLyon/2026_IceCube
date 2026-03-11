#include "subsystems/intake/pivot/PivotLogger.h"

PivotIOLogger::PivotIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isPivotMotorConnected(log, path + "/PivotMotor" + "/IO/isConnected"),
          pivotMotorCurrent(log, path + "/PivotMotor" + "/Current"),
          pivotMotorAppliedVoltage(log, path + "/PivotMotor" + "/AppliedVoltage"),
          pivotMotorBusVoltage(log, path + "/PivotMotor" + "/BusVoltage"),
          pivotMotorTemperature(log, path + "/PivotMotor" + "/Temperature"),
          isLeftEncoderConnected(log, path + "/LeftEncoder" + "/IO/isConnected"),
          pivotPos(log, path + "/PivotPos"),
          
          m_path(path)
{}

void PivotIOLogger::Log(const PivotIOInputs& inputs) {
    isPivotMotorConnected.Append(inputs.isPivotMotorConnected);
    pivotMotorCurrent.Append(inputs.pivotMotorCurrent);
    pivotMotorAppliedVoltage.Append(inputs.pivotMotorAppliedVoltage);
    pivotMotorBusVoltage.Append(inputs.pivotMotorBusVoltage);
    pivotMotorTemperature.Append(inputs.pivotMotorTemperature);
    isLeftEncoderConnected.Append(inputs.isLeftEncoderConnected);
    pivotPos.Append(inputs.pivotPos);
}