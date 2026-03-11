#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "PivotIO.h"

class PivotIOLogger {
public:
    PivotIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const PivotIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isPivotMotorConnected;
    wpi::log::DoubleLogEntry pivotMotorAppliedVoltage;
    wpi::log::DoubleLogEntry pivotMotorBusVoltage;
    wpi::log::DoubleLogEntry pivotMotorCurrent;
    wpi::log::DoubleLogEntry pivotMotorTemperature;
    wpi::log::DoubleLogEntry isLeftEncoderConnected;
    wpi::log::DoubleLogEntry pivotPos;

    const std::string& m_path;
};