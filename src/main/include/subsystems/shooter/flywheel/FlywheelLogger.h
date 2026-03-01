#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "FlywheelIO.h"

class FlywheelIOLogger {
public:
    FlywheelIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const FlywheelIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isLeftMotorConnected;
    wpi::log::DoubleLogEntry leftMotorCurrent;
    wpi::log::DoubleLogEntry leftMotorAppliedVoltage;
    wpi::log::DoubleLogEntry leftMotorBusVoltage;
    wpi::log::DoubleLogEntry leftMotorTemperature;
    wpi::log::DoubleLogEntry leftMotorInternalEncoderVelocity;

    wpi::log::BooleanLogEntry isRightMotorConnected;
    wpi::log::DoubleLogEntry rightMotorCurrent;
    wpi::log::DoubleLogEntry rightMotorAppliedVoltage;
    wpi::log::DoubleLogEntry rightMotorBusVoltage;
    wpi::log::DoubleLogEntry rightMotorTemperature;
    wpi::log::DoubleLogEntry rightMotorInternalEncoderVelocity;

    wpi::log::DoubleLogEntry shooterVelocity;

    const std::string& m_path;
};