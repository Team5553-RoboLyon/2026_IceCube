#include "subsystems/intake/intakeIOLogger.h"

IntakeIOLogger::IntakeIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isRollerMotorConnected(log, path + "/roller/rollerMotor" + "/isConnected"),
          rollerMotorCurrent(log, path + "/roller/rollerMotor" + "/Current"),
          rollerMotorAppliedVoltage(log, path + "/roller/rollerMotor" + "/AppliedVoltage"),
          rollerMotorBusVoltage(log, path + "/roller/rollerMotor" + "/BusVoltage"),
          rollerMotorTemperature(log, path + "/roller/rollerMotor" + "/Temperature"),
          isPivotMotorConnected(log, path + "/pivot/pivotMotor" + "/isConnected"),
          pivotMotorCurrent(log, path + "/pivot/pivotMotor" + "/Current"),
          pivotMotorAppliedVoltage(log, path + "/pivot/pivotMotor" + "/AppliedVoltage"),
          pivotMotorBusVoltage(log, path + "/pivot/pivotMotor" + "/BusVoltage"),
          pivotMotorTemperature(log, path + "pivot/pivotMotor" + "/Temperature"),
        //   isRightEncoderConnected(log, path + "/pivot/RightEncoder" + "/isConnected"),
          isLeftEncoderConnected(log, path + "/pivot/LeftEncoder" + "/isConnected"),
          pivotPos(log, path + "/pivot" + "/PivotPos"),
          
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
    isLeftEncoderConnected.Append(pivotInputs.isLeftEncoderConnected);
    // isRightEncoderConnected.Append(pivotInputs.isRightEncoderConnected);
    pivotPos.Append(pivotInputs.pivotPos);
}