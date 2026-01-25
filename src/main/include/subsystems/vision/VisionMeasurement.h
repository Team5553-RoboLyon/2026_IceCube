#pragma once

#include <frc/geometry/Pose2d.h>
#include <Eigen/Dense>

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
  Eigen::Vector3d stdDevs;

  VisionMeasurement() = default;

  VisionMeasurement(
      const frc::Pose2d &robotPose_,
      units::time::second_t timestamp_,
      const Eigen::Vector3d &stdDevs_)
      : robotPose(robotPose_), timestamp(timestamp_), stdDevs(stdDevs_) {}
};