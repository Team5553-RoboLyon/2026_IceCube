#include "subsystems/intake/intakeIOLogger.h"

IntakeIOLogger::IntakeIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isRollerMotorConnected(log, path + "/rollerMotor" + "/isConnected"),
          rollerMotorCurrent(log, path + "/rollerMotor" + "/Current"),
          rollerMotorAppliedVoltage(log, path + "/rollerMotor" + "/AppliedVoltage"),
          rollerMotorBusVoltage(log, path + "/rollerMotor" + "/BusVoltage"),
          rollerMotorTemperature(log, path + "/rollerMotor" + "/Temperature"),
          isPivotMotorConnected(log, path + "/pivotMotor" + "/isConnected"),
          pivotMotorCurrent(log, path + "/pivotMotor" + "/Current"),
          pivotMotorAppliedVoltage(log, path + "/pivotMotor" + "/AppliedVoltage"),
          pivotMotorBusVoltage(log, path + "/pivotMotor" + "/BusVoltage"),
          pivotMotorTemperature(log, path + "/pivotMotor" + "/Temperature"),
          
          
          m_path(path)
{}

void IntakeIOLogger::Log(const RollerIOInputs& rollerInputs, const PivotIOInputs& pivotInputs) {
        isRollerMotorConnected.Append(rollerInputs.isRollerMotorConnected);
    rollerMotorAppliedVoltage.Append(rollerInputs.rollerMotorAppliedVoltage);
    rollerMotorBusVoltage.Append(rollerInputs.rollerMotorBusVoltage);
    rollerMotorCurrent.Append(rollerInputs.rollerMotorCurrent);
    rollerMotorTemperature.Append(rollerInputs.rollerMotorTemperature);
    isPivotMotorConnected.Append(pivotInputs.isPivotMotorConnected);
    pivotMotorAppliedVoltage.Append(pivotInputs.pivotMotorAppliedVoltage);
    pivotMotorBusVoltage.Append(pivotInputs.pivotMotorBusVoltage);
    pivotMotorCurrent.Append(pivotInputs.pivotMotorCurrent);
    pivotMotorTemperature.Append(pivotInputs.pivotMotorTemperature);
    
}