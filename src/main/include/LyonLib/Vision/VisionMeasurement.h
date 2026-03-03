/*******************************************************************************
 * 
 * File        : VisionMeasurement.h (v1.0)
 * Library     : LyonLib (from 2026)
 * Description :  Defines the VisionMeasurement struct, which represents a 
 *                vision measurement containing the robot's pose, timestamp, 
 *                and standard deviations of the measurement.
 * 
 * Authors     : AKA (2026) and inspired by Team 4481
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <frc/geometry/Pose2d.h>
#include <wpi/array.h>
/**
 * A vision measurement.
 *
 * @param robotPose The robot's pose.
 * @param timestamp The timestamp of the measurement.
 * @param stdDevs The standard deviations of the measurement.
 */
struct VisionMeasurement {
  frc::Pose2d robotPose;
  units::time::second_t timestamp;
  const wpi::array<double, 3U> stdDevs;

  VisionMeasurement() = default;

  VisionMeasurement(
      const frc::Pose2d &robotPose_,
      units::time::second_t timestamp_,
      const wpi::array<double, 3U> &stdDevs_)
      : robotPose(robotPose_), timestamp(timestamp_), stdDevs(stdDevs_) {}
};