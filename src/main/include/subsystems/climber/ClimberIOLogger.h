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
        wpi::log::BooleanLogEntry isclimberMotorConnected;
    wpi::log::DoubleLogEntry climberMotorCurrent;
    wpi::log::DoubleLogEntry climberMotorAppliedVoltage;
    wpi::log::DoubleLogEntry climberMotorBusVoltage;
    wpi::log::DoubleLogEntry climberMotorTemperature;
    

    const std::string& m_path;
};