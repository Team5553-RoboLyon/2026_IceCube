// #pragma once

// #include "PhotonVisionIO.h"
// #include <photonvision/simulation/PhotonCameraSim.h>
// #include <photonvision/simulation/SimCameraProperties.h>
// #include <frc/apriltag/AprilTagFieldLayout.h>
// #include <frc/geometry/Transform3d.h>

// /**
//  * A class used to interface with a PhotonVision camera in simulation.
//  */
// class SimulatedPhotonVisionIO : public PhotonVisionIO {
//  public:
//   /**
//    * Create a new PhotonVisionIO object for a simulated robot.
//    *
//    * @param cameraName Name of the camera in NetworkTables.
//    * @param robotToCameraTransform Transform that represents the translation and
//    *     rotation from robot centre to the camera.
//    * @param aprilTagFieldLayout Layout of the AprilTags around the field.
//    * @param simCameraProperties Properties of the simulated PhotonVision camera.
//    */
//   SimulatedPhotonVisionIO(
//       const std::string& cameraName,
//       const frc::Transform3d& robotToCameraTransform,
//       const frc::AprilTagFieldLayout& aprilTagFieldLayout,
//       const photon::SimCameraProperties& simCameraProperties);
// };