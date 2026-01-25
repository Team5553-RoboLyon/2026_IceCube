#include "subsystems/vision/RealPhotonVisionIO.h"

RealPhotonVisionIO::RealPhotonVisionIO(
    const std::string& cameraName,
    const frc::Transform3d& robotToCameraTransform,
    const frc::AprilTagFieldLayout& aprilTagFieldLayout)
    : PhotonVisionIO(cameraName, robotToCameraTransform, aprilTagFieldLayout) {}