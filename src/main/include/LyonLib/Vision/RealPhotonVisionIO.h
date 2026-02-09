/*******************************************************************************
 * 
 * File        : RealPhotonVisionIO.h (v1.0)
 * Library     : LyonLib (from 2026)
 * Description : Concrete implementation of PhotonVisionIO for real robot usage, 
 *               interfacing with an actual PhotonVision camera and providing 
 *               vision data and pose estimation based on real AprilTag 
 *               detections. 
 *               This class handles the transformation of camera-space 
 *               measurements into field-space robot poses while managing latency 
 *               compensation through time-interpolatable buffers, specifically 
 *               designed for use in a real robot environment with 
 *               physical hardware constraints.
 * 
 * Authors     : AKA (2026) and inspired by Team 4481
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include "PhotonVisionIO.h"
#include <frc/geometry/Transform3d.h>

/**
 * A class used to interface with a PhotonVision camera on a real robot.
 */
class RealPhotonVisionIO : public PhotonVisionIO {
 public:
  /**
   * Create a new PhotonVisionIO object for a real robot.
   *
   * @param cameraName Name of the camera in NetworkTables.
   * @param robotToCameraTransform Transform that represents the translation and
   *     rotation from robot centre to the camera.
   * @param aprilTagFieldLayout Layout of the AprilTags around the field.
   */
  RealPhotonVisionIO(
      const std::string& cameraName,
      const frc::Transform3d& robotToCameraTransform,
      const frc::AprilTagFieldLayout& aprilTagFieldLayout);
};