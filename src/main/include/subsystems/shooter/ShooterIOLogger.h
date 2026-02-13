#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "hood/HoodIO.h"
#include "flywheel/FlywheelIO.h"

class ShooterIOLogger {
public:
    ShooterIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const HoodIOInputs& hoodInputs, const FlywheelIOInputs& flywheelInputs);

private:
    wpi::log::BooleanLogEntry isLeftMotorConnected;
    wpi::log::DoubleLogEntry LeftMotorCurrent;
    wpi::log::DoubleLogEntry LeftMotorAppliedVoltage;
    wpi::log::DoubleLogEntry LeftMotorBusVoltage;
    wpi::log::DoubleLogEntry LeftMotorTemperature;
    wpi::log::DoubleLogEntry LeftMotorEncoderVelocity;
    wpi::log::BooleanLogEntry isRightMotorConnected;
    wpi::log::DoubleLogEntry RightMotorCurrent;
    wpi::log::DoubleLogEntry RightMotorAppliedVoltage;
    wpi::log::DoubleLogEntry RightMotorBusVoltage;
    wpi::log::DoubleLogEntry RightMotorTemperature;
    wpi::log::DoubleLogEntry RightMotorEncoderVelocity;
    wpi::log::DoubleLogEntry ShooterVelocity;
    wpi::log::BooleanLogEntry isHoodMotorConnected;
    wpi::log::DoubleLogEntry HoodMotorCurrent;
    wpi::log::DoubleLogEntry HoodMotorAppliedVoltage;
    wpi::log::DoubleLogEntry HoodMotorBusVoltage;
    wpi::log::DoubleLogEntry HoodMotorTemperature;
    wpi::log::DoubleLogEntry HoodPos;

    const std::string& m_path;
};