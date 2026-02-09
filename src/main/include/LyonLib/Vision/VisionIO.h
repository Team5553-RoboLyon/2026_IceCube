/*******************************************************************************
 * 
 * File        : VisionIO.h (v1.0)
 * Library     : LyonLib (from 2026)
 * Description : Interface for a vision IO, defining the structure of the inputs and the methods to update them.
 * 
 * Authors     : AKA (2026) and inspired by Team 4481
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <array>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>

/**
 * Interface for a vision IO
 */
class VisionIO {
 public:
  /**
   * The inputs for a vision system
   */
  struct VisionInputs {
    bool connected = false;
    bool hasResults = false;

    /** Ambiguity ratio of the target, 0 is no ambiguity (good), 1 is max ambiguity (bad) */
    double ambiguityRatio = 0.0;

    std::vector<int64_t> visibleTagIDs;

    /** Two possible robot poses (best / alternate) */
    std::array<frc::Pose3d, 2> fieldSpaceRobotPoses{
        frc::Pose3d(), frc::Pose3d()};

    units::time::second_t timeStamp = 0.0_s;

    /** Area of the tags as a percentage of the screen taken up by the tag (0 - 100) */
    std::vector<double> tagAreas;

    std::string cameraName;
  };

  /**
   * Update the inputs of a vision system
   *
   * @param inputs the inputs to update
   */
  virtual void UpdateInputs(VisionInputs& inputs) = 0;

  /**
   * Pass the robot rotation measured with the IMU to the vision system.
   * This should be updated every loop.
   *
   * @param robotRotation Actual rotation of the robot
   */
  virtual void SetRobotRotation(const frc::Rotation2d& robotRotation) {}

  virtual ~VisionIO() = default;
};