#include "subsystems/turret/turretIOLogger.h"

TurretIOLogger::TurretIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               ismotorConnected(log, path + "/motor" + "/isConnected"),
          motorCurrent(log, path + "/motor" + "/Current"),
          motorAppliedVoltage(log, path + "/motor" + "/AppliedVoltage"),
          motorBusVoltage(log, path + "/motor" + "/BusVoltage"),
          motorTemperature(log, path + "/motor" + "/Temperature"),
                  orientation(log, path + "/encoder" + "/orientation"),
          m_path(path)
{}

void TurretIOLogger::Log(const TurretIOInputs& inputs) {
        ismotorConnected.Append(inputs.ismotorConnected);
    motorAppliedVoltage.Append(inputs.motorAppliedVoltage);
    motorBusVoltage.Append(inputs.motorBusVoltage);
    motorCurrent.Append(inputs.motorCurrent);
    motorTemperature.Append(inputs.motorTemperature);
        orientation.Append(inputs.orientation);
}