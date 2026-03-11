#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "RollerIO.h"

class RollerIOLogger {
public:
    RollerIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const RollerIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isRollerMotorConnected;
    wpi::log::DoubleLogEntry rollerMotorAppliedVoltage;
    wpi::log::DoubleLogEntry rollerMotorBusVoltage;
    wpi::log::DoubleLogEntry rollerMotorCurrent;
    wpi::log::DoubleLogEntry rollerMotorTemperature;

    const std::string& m_path;
};