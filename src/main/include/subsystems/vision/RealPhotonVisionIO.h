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