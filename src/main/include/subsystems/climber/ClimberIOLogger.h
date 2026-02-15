#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "climberIO.h"

class ClimberIOLogger {
public:
    ClimberIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const ClimberIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isMotorConnected;
    wpi::log::DoubleLogEntry motorCurrent;
    wpi::log::DoubleLogEntry motorAppliedVoltage;
    wpi::log::DoubleLogEntry motorBusVoltage;
    wpi::log::DoubleLogEntry motorTemperature;

    wpi::log::DoubleLogEntry hammerHeight;
    wpi::log::BooleanLogEntry irbreakerValue;
    wpi::log::BooleanLogEntry bottomLimitSwitchValue;
    

    const std::string& m_path;
};