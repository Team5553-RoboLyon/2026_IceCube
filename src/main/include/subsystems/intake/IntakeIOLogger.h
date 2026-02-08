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
    wpi::log::BooleanLogEntry isintakeMotorConnected;
    wpi::log::DoubleLogEntry intakeMotorCurrent;
    wpi::log::DoubleLogEntry intakeMotorAppliedVoltage;
    wpi::log::DoubleLogEntry intakeMotorBusVoltage;
    wpi::log::DoubleLogEntry intakeMotorTemperature;
    wpi::log::BooleanLogEntry ispivotMotorConnected;
    wpi::log::DoubleLogEntry pivotMotorCurrent;
    wpi::log::DoubleLogEntry pivotMotorAppliedVoltage;
    wpi::log::DoubleLogEntry pivotMotorBusVoltage;
    wpi::log::DoubleLogEntry pivotMotorTemperature;
    wpi::log::BooleanLogEntry isMichelMotorConnected;
    wpi::log::DoubleLogEntry michelMotorCurrent;
    wpi::log::DoubleLogEntry michelMotorAppliedVoltage;
    wpi::log::DoubleLogEntry michelMotorBusVoltage;
    wpi::log::DoubleLogEntry michelMotorTemperature;
    

    const std::string& m_path;
};