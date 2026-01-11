#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "intakeIO.h"

class IntakeIOLogger {
public:
    IntakeIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const IntakeIOInputs& inputs);

private:
        wpi::log::BooleanLogEntry isleftMotorConnected;
    wpi::log::DoubleLogEntry leftMotorCurrent;
    wpi::log::DoubleLogEntry leftMotorAppliedVoltage;
    wpi::log::DoubleLogEntry leftMotorBusVoltage;
    wpi::log::DoubleLogEntry leftMotorTemperature;
    wpi::log::BooleanLogEntry isrightMotorConnected;
    wpi::log::DoubleLogEntry rightMotorCurrent;
    wpi::log::DoubleLogEntry rightMotorAppliedVoltage;
    wpi::log::DoubleLogEntry rightMotorBusVoltage;
    wpi::log::DoubleLogEntry rightMotorTemperature;
    

    const std::string& m_path;
};