#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "turretIO.h"

class TurretIOLogger {
public:
    TurretIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const TurretIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry ismotorConnected;
    wpi::log::DoubleLogEntry motorCurrent;
    wpi::log::DoubleLogEntry motorAppliedVoltage;
    wpi::log::DoubleLogEntry motorBusVoltage;
    wpi::log::DoubleLogEntry motorTemperature;
    wpi::log::DoubleLogEntry orientation;

    const std::string& m_path;
};