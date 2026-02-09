/*******************************************************************************
 * 
 * File        : ComplexStructLogger.h (v1.0)
 * Library     : LyonLib (from 2026)
 * Description : NetworkTables NT4 structured logging utilities for WPILib.
 *               Provides generic, type-safe loggers for WPILib geometry and
 *               kinematics objects using NT4 Struct and StructArray topics.
 * 
 *               This library is designed to be fully compatible with
 *               AdvantageScope and future WPILib versions (2027+),
 *               avoiding deprecated SmartDashboard and untyped NetworkTables
 *               APIs.
 * 
 *               Supported logged types include:
 *                 - frc::Pose2d / frc::Pose3d
 *                 - frc::Rotation2d / frc::Rotation3d
 *                 - frc::Transform2d / frc::Transform3d
 *                 - frc::DifferentialSample[]
 *                 - frc::Trajectory
 * 
 *               Publishers are created once and updated periodically, ensuring
 *               deterministic behavior, minimal allocations, and NT4 replay
 *               compatibility.
 * 
 * Authors     : AKA (2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <span>
#include <string_view>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>


/**
 * @brief Generic logger for a single NT4 Struct topic.
 * 
 * @tparam T Type of the WPILib object to log (Pose2d, Pose3d, Rotation2d, Rotation3d,
 *           Transform2d, Transform3d, Trajectory, etc.). Must have a Struct schema defined.
 * 
 * @note Publishes to NetworkTables using NT4 StructTopic.
 *       AdvantageScope will automatically recognize the topic type.
 */
template <typename T>
class StructLogger {
public:
    /**
   * @brief Constructs a StructLogger and initializes the NT4 Struct publisher.
   * 
   * @param topicName The fully qualified NT4 topic name to publish to.
   */
  explicit StructLogger(std::string_view topicName) {
    auto inst = nt::NetworkTableInstance::GetDefault();
    m_publisher = inst.GetStructTopic<T>(topicName).Publish();
  }

  /**
   * @brief Publishes a single value of type T to the NT4 topic.
   * 
   * @param value The object to log (Pose2d, Pose3d, etc.).
   * 
   * @note Should be called periodically (e.g. in subsystem Periodic()).
   *       Does not allocate memory on each call.
   */
  void Log(const T& value) {
    m_publisher.Set(value);
  }

private:
  nt::StructPublisher<T> m_publisher; //< Underlying NT4 Struct publisher
};



/*******************************************************************************/



/**
 * @brief Generic logger for an array of NT4 Struct objects.
 * 
 * @tparam T Type of the WPILib object array to log (DifferentialSample, Pose2d[], etc.).
 * 
 * @note Publishes to NetworkTables using NT4 StructArrayTopic.
 *       AdvantageScope will automatically recognize the array topic type.
 */
template <typename T>
class StructArrayLogger {
public:
/**
   * @brief Constructs a StructArrayLogger and initializes the NT4 StructArray publisher.
   * 
   * @param topicName The fully qualified NT4 topic name to publish the array to.
   */
  explicit StructArrayLogger(std::string_view topicName) {
    auto inst = nt::NetworkTableInstance::GetDefault();
    m_publisher = inst.GetStructArrayTopic<T>(topicName).Publish();
  }

  /**
   * @brief Publishes an array of objects to the NT4 topic.
   * 
   * @param values A span of objects to log. Can be a std::array, std::vector, or any contiguous container.
   * 
   * @note Should be called periodically (e.g. in subsystem Periodic()).
   *       Avoid large allocations for performance.
   */
  void Log(std::span<const T> values) {
    m_publisher.Set(values);
  }

private:
  nt::StructArrayPublisher<T> m_publisher; //< Underlying NT4 StructArray publisher
};


/*******************************************************************************
 * Usage Example:
 *
 * // Pose2d single object
 * StructLogger<frc::Pose2d> robotPoseLogger("/Pose/Robot");
 *
 * // Pose2d array for multiple estimated positions
 * StructArrayLogger<frc::Pose2d> poseArrayLogger("/Pose/RobotArray");
 *
 * void Periodic() {
 *   frc::Pose2d pose = odometry.GetPose();
 *   robotPoseLogger.Log(pose);
 *
 *   std::array<frc::Pose2d, 2> poses{pose1, pose2};
 *   poseArrayLogger.Log(poses);
 * }
 *******************************************************************************/