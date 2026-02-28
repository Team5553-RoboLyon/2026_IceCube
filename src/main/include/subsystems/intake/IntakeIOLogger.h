#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "roller/RollerIO.h"
#include "pivot/PivotIO.h"

class IntakeIOLogger {
public:
    IntakeIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const RollerIOInputs& rollerInputs, const PivotIOInputs& pivotInputs);

private:
    wpi::log::BooleanLogEntry isRollerMotorConnected;
    wpi::log::DoubleLogEntry rollerMotorCurrent;
    wpi::log::DoubleLogEntry rollerMotorAppliedVoltage;
    wpi::log::DoubleLogEntry rollerMotorBusVoltage;
    wpi::log::DoubleLogEntry rollerMotorTemperature;
    wpi::log::BooleanLogEntry isPivotMotorConnected;
    wpi::log::DoubleLogEntry pivotMotorCurrent;
    wpi::log::DoubleLogEntry pivotMotorAppliedVoltage;
    wpi::log::DoubleLogEntry pivotMotorBusVoltage;
    wpi::log::DoubleLogEntry pivotMotorTemperature;
    wpi::log::BooleanLogEntry isRightEncoderConnected;
    wpi::log::BooleanLogEntry isLeftEncoderConnected;
    wpi::log::DoubleLogEntry pivotPos;

    const std::string& m_path;
};