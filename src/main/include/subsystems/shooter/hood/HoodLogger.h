#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "HoodIO.h"

class HoodIOLogger {
public:
    HoodIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const HoodIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isMotorConnected;
    wpi::log::DoubleLogEntry motorCurrent;
    wpi::log::DoubleLogEntry motorAppliedVoltage;
    wpi::log::DoubleLogEntry motorBusVoltage;
    wpi::log::DoubleLogEntry motorTemperature;

    wpi::log::DoubleLogEntry hoodAngle;

    const std::string& m_path;
};