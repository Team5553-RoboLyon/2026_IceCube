#include "subsystems/drivetrain/DrivetrainIOLogger.h"

DrivetrainIOLogger::DrivetrainIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isFrontLeftMotorConnected(log, path + "/Motors/FrontLeft/isConnected"),
          isBackLeftMotorConnected(log, path + "/Motors/BackLeft/isConnected"),
          isFrontRightMotorConnected(log, path + "/Motors/FrontRight/isConnected"),
          isBackRightMotorConnected(log, path + "/Motors/BackRight/isConnected"),
          frontLeftMotorAppliedVoltage(log, path + "/Motors/FrontLeft/appliedVoltage"),
          frontLeftMotorBusVoltage(log, path + "/Motors/FrontLeft/busVoltage"),
          frontLeftMotorCurrent(log, path + "/Motors/FrontLeft/current"),
          frontLeftMotorTemperature(log, path + "/Motors/FrontLeft/temperature"),
          backLeftMotorAppliedVoltage(log, path + "/Motors/BackLeft/appliedVoltage"),
          backLeftMotorBusVoltage(log, path + "/Motors/BackLeft/busVoltage"),
          backLeftMotorCurrent(log, path + "/Motors/BackLeft/current"),
          backLeftMotorTemperature(log, path + "/Motors/BackLeft/temperature"),
          frontRightMotorAppliedVoltage(log, path + "/Motors/FrontRight/appliedVoltage"),
          frontRightMotorBusVoltage(log, path + "/Motors/FrontRight/busVoltage"),
          frontRightMotorCurrent(log, path + "/Motors/FrontRight/current"),
          frontRightMotorTemperature(log, path + "/Motors/FrontRight/temperature"),
          backRightMotorAppliedVoltage(log, path + "/Motors/BackRight/appliedVoltage"),
          backRightMotorBusVoltage(log, path + "/Motors/BackRight/busVoltage"),
          backRightMotorCurrent(log, path + "/Motors/BackRight/current"),
          backRightMotorTemperature(log, path + "/Motors/BackRight/temperature"),
          leftSideTraveledDistance(log, path + "/LeftSide/traveledDistance"),
          leftSideVelocity(log, path + "/LeftSide/velocity"),
          rightSideTraveledDistance(log, path + "/RightSide/traveledDistance"),
          rightSideVelocity(log, path + "/RightSide/velocity"),
          odometryPosition(log, path + "/Robot/position")
{}

void DrivetrainIOLogger::Log(const DrivetrainIOInputs& inputs) {
    isFrontLeftMotorConnected.Append(inputs.isFrontLeftMotorConnected);
    isBackLeftMotorConnected.Append(inputs.isBackLeftMotorConnected);
    isFrontRightMotorConnected.Append(inputs.isFrontRightMotorConnected);
    isBackRightMotorConnected.Append(inputs.isBackRightMotorConnected);
    frontLeftMotorAppliedVoltage.Append(inputs.frontLeftMotorAppliedVoltage.value());
    frontLeftMotorBusVoltage.Append(inputs.frontLeftMotorBusVoltage.value());
    frontLeftMotorCurrent.Append(inputs.frontLeftMotorCurrent.value());
    frontLeftMotorTemperature.Append(inputs.frontLeftMotorTemperature.value());
    backLeftMotorAppliedVoltage.Append(inputs.backLeftMotorAppliedVoltage.value());
    backLeftMotorBusVoltage.Append(inputs.backLeftMotorBusVoltage.value());
    backLeftMotorCurrent.Append(inputs.backLeftMotorCurrent.value());
    backLeftMotorTemperature.Append(inputs.backLeftMotorTemperature.value());
    frontRightMotorAppliedVoltage.Append(inputs.frontRightMotorAppliedVoltage.value());
    frontRightMotorBusVoltage.Append(inputs.frontRightMotorBusVoltage.value());
    frontRightMotorCurrent.Append(inputs.frontRightMotorCurrent.value());
    frontRightMotorTemperature.Append(inputs.frontRightMotorTemperature.value());
    backRightMotorAppliedVoltage.Append(inputs.backRightMotorAppliedVoltage.value());
    backRightMotorBusVoltage.Append(inputs.backRightMotorBusVoltage.value());
    backRightMotorCurrent.Append(inputs.backRightMotorCurrent.value());
    backRightMotorTemperature.Append(inputs.backRightMotorTemperature.value());
    leftSideTraveledDistance.Append(inputs.leftSideTraveledDistance.value());
    leftSideVelocity.Append(inputs.leftSideVelocity.value());
    rightSideTraveledDistance.Append(inputs.rightSideTraveledDistance.value());
    rightSideVelocity.Append(inputs.rightSideVelocity.value());
    odometryPosition.Append(inputs.odometryPosition);
}