#pragma once


#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>


#include "LyonLib/logging/Alert.h"

#include <units/angle.h>

#include "VisionIO.h"

class PhotonVisionIO : public VisionIO {
 public:
  PhotonVisionIO(
      const std::string& cameraName,
      const frc::Transform3d& robotToCameraTransform,
      const frc::AprilTagFieldLayout& aprilTagFieldLayout);

    ~PhotonVisionIO() noexcept override = default;
  void UpdateInputs(VisionInputs& inputs) override;

    /**
     * Stores the current robot rotation into a time-interpolatable buffer.
     *
     * This buffer is used later to estimate the robot's orientation at the exact
     * timestamp when a vision frame was captured. Vision processing introduces
     * latency, so the rotation used for pose estimation must correspond to the
     * robot's state at the moment of capture, not at the moment the result is
     * processed.
     *
     * @param robotRotation The current robot yaw (typically from the gyro).
     *
     * The timestamp is computed using the FPGA timer in microseconds and converted
     * to seconds. A fixed latency offset of 40ms is subtracted to approximate the
     * time when the camera image was actually captured.
     *
     * Note:
     * - The 40ms offset is an estimate and should be calibrated.
     */
  void SetRobotRotation(const frc::Rotation2d& robotRotation);

 private:

    /**
     * Computes two possible robot poses from a single AprilTag detection.
     *
     * PhotonVision can return two candidate camera-to-target solutions when solving
     * PnP with a single AprilTag (due to pose ambiguity). This method uses the
     * robot yaw from the gyro (interpolated to the image capture timestamp) to
     * resolve the rotational ambiguity and produce two field-space robot pose
     * hypotheses.
     *
     * @param latestResult The latest PhotonPipelineResult containing the AprilTag
     *                     detection and PnP solutions.
     * @return An array of two possible robot poses in field coordinates.
     *         If the AprilTag is not found in the field layout or the gyro
     *         timestamp cannot be interpolated, both poses will be the default
     *         (identity) Pose3d.
     */
  std::array<frc::Pose3d, 2> RetrieveSingleTagEstimates(const photon::PhotonPipelineResult& latestResult);

  /**
 * Computes robot poses using a multi-tag AprilTag detection.
 *
 * When multiple AprilTags are visible in the same frame, PhotonVision can solve
 * PnP using all observed tags at once and returns a single best pose estimate
 * and an alternate pose estimate. This method converts those camera-space
 * transforms into field-space robot poses by applying the known transform from
 * camera to robot.
 *
 * @param latestResult The latest PhotonPipelineResult containing the multi-tag
 *                     pose estimate.
 * @return An array of two possible robot poses in field coordinates:
 *         - Index 0: Best pose estimate.
 *         - Index 1: Alternate pose estimate.
 *         If no multi-tag estimate is available, both poses will default to the
 *         identity Pose3d.
 */
  std::array<frc::Pose3d, 2> RetrieveMultiTagEstimates(const photon::PhotonPipelineResult& latestResult);

  /**
 * Logs the difference between the expected camera rotation and the observed
 * camera rotation from vision.
 *
 * This method compares the rotation of the camera as computed from the latest
 * vision estimate (fieldSpaceCameraPose) against the known camera rotation
 * derived from the camera-to-robot transform. The difference is recorded to
 * SmartDashboard for debugging and calibration purposes.
 *
 * The values logged are:
 *  - Roll error (x-axis rotation)
 *  - Pitch error (y-axis rotation)
 *
 * @param fieldSpaceCameraPose The camera pose in field coordinates as estimated
 *                             by vision (field → camera).
 */
  void LogRotationDiff(const frc::Pose3d& fieldSpaceCameraPose);

 private:
  photon::PhotonCamera m_camera;
  frc::AprilTagFieldLayout m_aprilTagFieldLayout;
  frc::Transform3d m_cameraToRobotTransform;
  std::string m_cameraName;

  static constexpr double kRotationLoopsDelay = 2.0;

  Alert m_nullPointerAlertRBL{"Vision/PhotonVisionIO Null Pointer",
                                 "PhotonVisionIO received a null pointer "
                                 "when attempting to access the latest "
                                 "camera result.",
                                 Alert::AlertType::ERROR};

  frc::TimeInterpolatableBuffer<frc::Rotation2d> m_rotationBuffer{units::second_t{1.5}};
};