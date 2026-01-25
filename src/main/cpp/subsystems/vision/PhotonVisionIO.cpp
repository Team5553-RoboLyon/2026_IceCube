#include "subsystems/vision/PhotonVisionIO.h"

#include <LyonLib/utils/TimerRBL.h>

#include <frc/smartdashboard/SmartDashboard.h>

PhotonVisionIO::PhotonVisionIO(
    const std::string& cameraName,
    const frc::Transform3d& robotToCameraTransform,
    const frc::AprilTagFieldLayout& aprilTagFieldLayout)
    : m_camera(cameraName),
      m_aprilTagFieldLayout(aprilTagFieldLayout),
      m_cameraToRobotTransform(robotToCameraTransform.Inverse()),
      m_cameraName(cameraName),
      m_nullPointerAlertRBL(
          "Alerts/Vision",
          "[Camera " + cameraName + "] has caused null pointer exceptions",
          Alert::AlertType::ERROR) {
  m_nullPointerAlertRBL.Set(false);
}

void PhotonVisionIO::UpdateInputs(VisionInputs& inputs) {
  // Reset outputs every cycle
  inputs.hasResults = false;
  inputs.connected = false;

    // Store camera name for logging / debugging
    inputs.cameraName = m_cameraName;

    // Check camera connection
    if (!m_camera.IsConnected()) {
      return;
    }

    inputs.connected = true;

    // Get all unread vision results
    std::vector<photon::PhotonPipelineResult> results = m_camera.GetAllUnreadResults();

    // No results or no detected targets
    if (results.empty() || !results.back().HasTargets()) {
      return;
    }

    inputs.hasResults = true;

    // Use the most recent result to minimize latency
    const auto& latestResult = results.back();
    const auto& visibleTags = latestResult.GetTargets();

    // Pose ambiguity from best target (single-tag heuristic)
    inputs.ambiguityRatio = latestResult.GetBestTarget().GetPoseAmbiguity();

    // Extract tag areas and IDs
    inputs.tagAreas.resize(visibleTags.size());
    inputs.visibleTagIDs.resize(visibleTags.size());

    for (size_t i = 0; i < visibleTags.size(); ++i) 
    {
      inputs.tagAreas[i] = visibleTags[i].GetArea();
      inputs.visibleTagIDs[i] = visibleTags[i].GetFiducialId();
    }

    // Timestamp of the camera frame
    inputs.timeStamp = latestResult.GetTimestamp();

    // Choose estimation strategy based on tag count
    if (visibleTags.size() == 1) 
    {
      inputs.fieldSpaceRobotPoses = RetrieveSingleTagEstimates(latestResult);
    } 
    else 
    {
      inputs.fieldSpaceRobotPoses = RetrieveMultiTagEstimates(latestResult);
    }    
}

void PhotonVisionIO::SetRobotRotation(const frc::Rotation2d& robotRotation) {
  units::time::second_t timestamp = units::time::second_t(TimerRBL::GetFPGATimestampInMicroSeconds() * 1e-6 - 0.02 * kRotationLoopsDelay);
  m_rotationBuffer.AddSample(timestamp, robotRotation);
}


// Computes robot poses using a single AprilTag detection.
// Uses gyro yaw to remove rotational ambiguity.
std::array<frc::Pose3d, 2> PhotonVisionIO::RetrieveSingleTagEstimates(const photon::PhotonPipelineResult& latestResult) {

    // Two possible robot poses due to PnP ambiguity
    std::array<frc::Pose3d, 2> possibleRobotPoses{frc::Pose3d(), frc::Pose3d()};

    const auto& target = latestResult.GetBestTarget();

    // Get known field pose of the detected AprilTag
    auto tagPoseOpt = m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId());

    // Get robot yaw at the exact capture timestamp
    auto robotRotOpt = m_rotationBuffer.Sample(latestResult.GetTimestamp());

    if (tagPoseOpt && robotRotOpt) 
    {
        const frc::Pose3d& tagPose = *tagPoseOpt;

        // Rotation of the tag in field coordinates
        frc::Rotation3d tagRotation = tagPose.Rotation();

        // Robot yaw from gyro / odometry
        frc::Rotation2d robotRotation = *robotRotOpt;

        // Compute robot-to-tag rotation using known yaw
        frc::Rotation3d robotToTargetRot = tagRotation - frc::Rotation3d(0_deg, 0_deg, robotRotation.Radians());

        // Convert to camera-to-target rotation
        frc::Rotation3d cameraToTargetRot = robotToTargetRot + m_cameraToRobotTransform.Rotation();

        // Build two possible camera-to-target transforms
        std::array<frc::Transform3d, 2> camToTargetOptions{frc::Transform3d(),frc::Transform3d()};



        // Check that both PnP solutions provide valid translations.
        // PhotonVision returns two possible camera-to-target transforms
        // when solving PnP with a single AprilTag.
        auto bestTrans = target.GetBestCameraToTarget().Translation();
        auto altTrans  = target.GetAlternateCameraToTarget().Translation();

        // Validate translations by ensuring each component is finite
        bool bestValid = std::isfinite(bestTrans.X().value()) &&
                        std::isfinite(bestTrans.Y().value()) &&
                        std::isfinite(bestTrans.Z().value());

        bool altValid  = std::isfinite(altTrans.X().value()) &&
                        std::isfinite(altTrans.Y().value()) &&
                        std::isfinite(altTrans.Z().value());

        // If both are valid, build the transform options
        if (bestValid && altValid) 
        {
            // Build two camera-to-target transforms using:
            //  - The translation from PhotonVision's PnP solution
            //  - The rotation we reconstructed using the robot's gyro yaw
            camToTargetOptions = {
                frc::Transform3d(bestTrans, cameraToTargetRot),
                frc::Transform3d(altTrans, cameraToTargetRot)
            };
        }

        // Convert each camera-to-target hypothesis into a field-space robot pose.
        // PhotonUtils handles the full chain:
        //   Field → Tag → Camera → Robot
        for (size_t i = 0; i < camToTargetOptions.size(); ++i) 
        {
            // Convert camera->target transform into field->robot pose:
            //  field->robot = field->tag * (camera->tag)^-1 * camera->robot
            //
            // This is equivalent to:
            //  1) transform the tag pose into camera pose
            //  2) then apply the camera->robot offset
            //
            // This is the 3D equivalent of PhotonUtils::EstimateFieldToRobot (2D).
            possibleRobotPoses[i] = tagPose
            .TransformBy(camToTargetOptions[i].Inverse())
            .TransformBy(m_cameraToRobotTransform);
        }

        // Log camera rotation error for debugging
        LogRotationDiff(tagPose.TransformBy(target.GetBestCameraToTarget().Inverse()));
    }

  return possibleRobotPoses;
}

std::array<frc::Pose3d, 2> PhotonVisionIO::RetrieveMultiTagEstimates(const photon::PhotonPipelineResult& latestResult) {

  frc::Transform3d bestMultiPose;
  frc::Transform3d altMultiPose;

  if (latestResult.MultiTagResult()) {
    const auto& estimate = latestResult.MultiTagResult()->estimatedPose;
    bestMultiPose = estimate.best;
    altMultiPose = estimate.alt;
  }

  frc::Pose3d basePose;

  LogRotationDiff(basePose.TransformBy(bestMultiPose));

  return {
      basePose.TransformBy(bestMultiPose)
          .TransformBy(m_cameraToRobotTransform),
      basePose.TransformBy(altMultiPose)
          .TransformBy(m_cameraToRobotTransform)};
}

void PhotonVisionIO::LogRotationDiff(const frc::Pose3d& fieldSpaceCameraPose) {

  frc::Rotation3d rotation = fieldSpaceCameraPose.Rotation();
  frc::Rotation3d cameraRotation = m_cameraToRobotTransform.Inverse().Rotation();

  frc::SmartDashboard::PutNumber("Vision/Camera Rotation Error/" + m_cameraName + "/x (roll)",
                                    (cameraRotation.X() - rotation.X()).value());

  frc::SmartDashboard::PutNumber(
      "Vision/Camera Rotation Error/" + m_cameraName + "/y (pitch)",
      (cameraRotation.Y() - rotation.Y()).value());
}