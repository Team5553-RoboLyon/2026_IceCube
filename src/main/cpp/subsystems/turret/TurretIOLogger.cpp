#include "subsystems/turret/turretIOLogger.h"

TurretIOLogger::TurretIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               ismotorConnected(log, path + "/Motor" + "/isConnected"),
          motorCurrent(log, path + "/Motor" + "/Current"),
          motorAppliedVoltage(log, path + "/Motor" + "/AppliedVoltage"),
          motorBusVoltage(log, path + "/Motor" + "/BusVoltage"),
          motorTemperature(log, path + "/Motor" + "/Temperature"),
          orientation(log, path + "/Sensors" + "/orientation"),
          hallEffectSensorValue(log, path + "/Sensors" + "/hallEffectSensorValue"),

          m_path(path)
{}

void TurretIOLogger::Log(const TurretIOInputs& inputs) {
    ismotorConnected.Append(inputs.isMotorConnected);
    motorAppliedVoltage.Append(inputs.motorAppliedVoltage);
    motorBusVoltage.Append(inputs.motorBusVoltage);
    motorCurrent.Append(inputs.motorCurrent);
    motorTemperature.Append(inputs.motorTemperature);
    orientation.Append(inputs.orientation);
    hallEffectSensorValue.Append(inputs.hallEffectSensorValue);
}