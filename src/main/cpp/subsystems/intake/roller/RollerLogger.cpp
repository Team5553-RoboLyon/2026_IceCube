#include "subsystems/intake/roller/RollerLogger.h"

RollerIOLogger::RollerIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isRollerMotorConnected(log, path + "/RollerMotor" + "/IO/isConnected"),
          rollerMotorCurrent(log, path + "/RollerMotor" + "/Current"),
          rollerMotorAppliedVoltage(log, path + "/RollerMotor" + "/AppliedVoltage"),
          rollerMotorBusVoltage(log, path + "/RollerMotor" + "/BusVoltage"),
          rollerMotorTemperature(log, path + "/RollerMotor" + "/Temperature"),
          
          m_path(path)
{}

void RollerIOLogger::Log(const RollerIOInputs& inputs) {
    isRollerMotorConnected.Append(inputs.isRollerMotorConnected);
    rollerMotorCurrent.Append(inputs.rollerMotorCurrent);
    rollerMotorAppliedVoltage.Append(inputs.rollerMotorAppliedVoltage);
    rollerMotorBusVoltage.Append(inputs.rollerMotorBusVoltage);
    rollerMotorTemperature.Append(inputs.rollerMotorTemperature);
}