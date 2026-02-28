#pragma once
#include "frc/kinematics/ChassisSpeeds.h"

#include "frc/geometry/Pose2d.h"
#include "units/voltage.h"
#include "units/velocity.h"
#include "units/current.h"
#include "units/temperature.h"
#include "units/length.h"

struct DrivetrainIOInputs
{
    bool isFrontLeftMotorConnected = true;
    bool isBackLeftMotorConnected = true;
    bool isFrontRightMotorConnected = true;
    bool isBackRightMotorConnected = true;

    units::volt_t frontLeftMotorAppliedVoltage = 0.0_V;
    units::volt_t frontLeftMotorBusVoltage = 0.0_V;
    units::ampere_t frontLeftMotorCurrent = 0.0_A;
    units::celsius_t frontLeftMotorTemperature = 0.0_degC;

    units::volt_t backLeftMotorAppliedVoltage = 0.0_V;
    units::volt_t backLeftMotorBusVoltage = 0.0_V;
    units::ampere_t backLeftMotorCurrent = 0.0_A;
    units::celsius_t backLeftMotorTemperature = 0.0_degC;

    units::volt_t frontRightMotorAppliedVoltage = 0.0_V;
    units::volt_t frontRightMotorBusVoltage = 0.0_V;
    units::ampere_t frontRightMotorCurrent = 0.0_A;
    units::celsius_t frontRightMotorTemperature = 0.0_degC;

    units::volt_t backRightMotorAppliedVoltage = 0.0_V;
    units::volt_t backRightMotorBusVoltage = 0.0_V;
    units::ampere_t backRightMotorCurrent = 0.0_A;
    units::celsius_t backRightMotorTemperature = 0.0_degC;

    units::meter_t leftSideTraveledDistance = 0.0_m;
    units::meters_per_second_t leftSideVelocity = 0.0_mps;
    units::meter_t rightSideTraveledDistance = 0.0_m;
    units::meters_per_second_t rightSideVelocity = 0.0_mps;

    frc::Pose2d odometryPosition;
};


class DrivetrainIO {
public:
    virtual ~DrivetrainIO() = default;

    virtual void UpdateInputs(DrivetrainIOInputs& inputs) = 0;

    virtual void SetVoltage(const units::volt_t leftSideVoltage, const units::volt_t rightSideVoltage) = 0;
    virtual void SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle) = 0;
    
    virtual void SetChassisSpeed(const frc::ChassisSpeeds &speeds) = 0;

    virtual void ResetPosition(const frc::Pose2d& position) = 0;
};