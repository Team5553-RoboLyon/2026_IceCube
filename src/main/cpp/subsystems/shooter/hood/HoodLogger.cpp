#include "subsystems/shooter/hood/HoodLogger.h"

HoodIOLogger::HoodIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isMotorConnected(log, path + "/Motor" + "/IO/isConnected"),
          motorCurrent (log, path + "/Motor" + "/Current"),
          motorAppliedVoltage(log, path + "/Motor" + "/AppliedVoltage"),
          motorBusVoltage(log, path + "/Motor" + "/BusVoltage"),
          motorTemperature(log, path + "/Motor" + "/Temperature"),
          hoodAngle(log, path + "/HoodAngle"),
          m_path(path)
{}

void HoodIOLogger::Log(const HoodIOInputs& inputs) {
    isMotorConnected.Append(inputs.isMotorConnected);
    motorCurrent.Append(inputs.motorCurrent);
    motorAppliedVoltage.Append(inputs.motorAppliedVoltage);
    motorBusVoltage.Append(inputs.motorBusVoltage);
    motorTemperature.Append(inputs.motorTemperature);
    hoodAngle.Append(inputs.hoodAngle);
}