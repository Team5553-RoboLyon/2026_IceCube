#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "indexerIO.h"

class IndexerIOLogger {
public:
    IndexerIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const IndexerIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isindexerMotorConnected;
    wpi::log::DoubleLogEntry indexerMotorCurrent;
    wpi::log::DoubleLogEntry indexerMotorAppliedVoltage;
    wpi::log::DoubleLogEntry indexerMotorBusVoltage;
    wpi::log::DoubleLogEntry indexerMotorTemperature;
    wpi::log::BooleanLogEntry isClodeMotorConnected;
    wpi::log::DoubleLogEntry clodeMotorCurrent;
    wpi::log::DoubleLogEntry clodeMotorAppliedVoltage;
    wpi::log::DoubleLogEntry clodeMotorBusVoltage;
    wpi::log::DoubleLogEntry clodeMotorTemperature;

    const std::string& m_path;
};