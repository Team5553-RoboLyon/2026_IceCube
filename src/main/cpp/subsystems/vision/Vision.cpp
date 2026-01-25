#include "subsystems/vision/Vision.h"

Vision::Vision(const VisionFilterParameters& filterParameters,
               std::vector<std::shared_ptr<VisionIO>> ios)
    : m_filterParameters(filterParameters),
      m_visionIOs(std::move(ios)) {

  m_inputs.resize(m_visionIOs.size());
  m_connectionAlerts.reserve(m_visionIOs.size());

  for (size_t i = 0; i < m_visionIOs.size(); ++i) {
    m_connectionAlerts.emplace_back(
        "Alerts/Vision/" + std::to_string(i),
        "[Camera " + std::to_string(i) + "] disconnected",
        Alert::AlertType::WARNING);
  }

//   nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();

//   m_acceptedPosePubs.reserve(m_visionIOs.size());
//   m_tagPosesPubs.reserve(m_visionIOs.size());
//   m_distancePubs.reserve(m_visionIOs.size());
//   m_stdDevPubs.reserve(m_visionIOs.size());

//   for (size_t i = 0; i < m_visionIOs.size(); ++i) {
//     m_acceptedPosePubs.push_back(
//         ntInst.GetStructTopic("Vision/Accepted Poses/" + m_visionIOs[i]->GetName(),
//                               frc::Pose3d::)
//             .Publish());

//     m_tagPosesPubs.push_back(
//         ntInst.GetStructTopic("Vision/Tag Poses/" + m_visionIOs[i]->GetName(),
//                               frc::Pose3d::kPose3dStruct)
//             .Publish());

//     m_distancePubs.push_back(
//         ntInst.GetDoubleTopic("Vision/Estimated Distance/" + m_visionIOs[i]->GetName())
//             .Publish());

//     m_stdDevPubs.push_back(
//         ntInst.GetStructTopic("Vision/Standard Deviations/" + m_visionIOs[i]->GetName(),
//                               nt::Value::MakeDoubleArray({}))
//             .Publish());
//   }
}


void Vision::Periodic() {
  for (size_t i = 0; i < m_visionIOs.size(); ++i) {
    m_visionIOs[i]->UpdateInputs(m_inputs[i]);
    m_visionIOs[i]->SetRobotRotation(m_actualRobotPose.Rotation());

    // Error handling for the vision IO
    m_connectionAlerts[i].Set(!m_inputs[i].connected);
    frc::SmartDashboard::PutBoolean(
        "Connection Status/Camera " + m_inputs[i].cameraName,
        m_inputs[i].connected);

    // Logger::ProcessInputs("Vision/" + m_inputs[i].cameraName + " (" + std::to_string(i) + ")",
    //                       m_inputs[i]);
  }
}

frc2::CommandPtr Vision::ProcessVision(
    std::function<frc::Pose2d()> robotPoseSupplier,
    std::function<void(const VisionMeasurement&)> measurementConsumer) {

  return frc2::cmd::Run(
      [this, robotPoseSupplier, measurementConsumer] {
        m_actualRobotPose = robotPoseSupplier();

        for (size_t i = 0; i < m_visionIOs.size(); ++i) {
          if (!m_inputs[i].connected || !m_inputs[i].hasResults) {
            m_disconnectCount[i]++;

            if (m_disconnectCount[i] > m_maxDisconnectCount) {
            //   Logger::RecordOutput("Vision/Accepted Poses/" + m_inputs[i].cameraName,
            //                        m_defaultLogPos);
                
            }
            continue;
          }
          m_disconnectCount[i] = 0;

          // Log all tag poses that are seen
          std::vector<frc::Pose3d> tagPoses;
          for (double id : m_inputs[i].visibleTagIDs) {
            auto tagPose = m_filterParameters.aprilTagFieldLayout.GetTagPose(static_cast<int>(id));
            if (tagPose) {
              tagPoses.push_back(*tagPose);
            }
          }

        //   Logger::RecordOutput("Vision/Tag Poses/" + m_inputs[i].cameraName,
        //                        tagPoses);

          // If ambiguity ratio is too high, continue
          if (m_inputs[i].ambiguityRatio > m_filterParameters.maxAmbiguityRatio) {
            continue;
          }

          // Select the pose closest to the actual robot pose
          frc::Pose3d selectedPose;
          if (m_inputs[i].ambiguityRatio < m_filterParameters.maxAmbiguityRatio / 2) {
            selectedPose = m_inputs[i].fieldSpaceRobotPoses[0];
          } else {
            selectedPose = SelectClosestPose(m_inputs[i].fieldSpaceRobotPoses,
                                            m_actualRobotPose);
          }

          if (OutsideFieldBounds(selectedPose)) {
            continue;
          }

          double tagDistance = CalculateTagDistance(m_inputs[i].tagAreas);
        //   Logger::RecordOutput("Vision/Estimated Distance/" + m_inputs[i].cameraName,
        //                        tagDistance);

          if (tagDistance > m_filterParameters.maxAprilTagDistance) {
            continue;
          }

            Eigen::Vector3d stdDevMat = DetermineStandardDeviation(
              tagDistance,
              m_inputs[i].visibleTagIDs.size() > 1,
              m_inputs[i].visibleTagIDs.size());

          units::time::second_t timestamp = m_inputs[i].timeStamp;

        //   Logger::RecordOutput("Vision/Accepted Poses/" + m_inputs[i].cameraName,
        //                        selectedPose);
        //   Logger::RecordOutput("Vision/Standard Deviations/" + m_inputs[i].cameraName,
        //                        stdDevMat);

          measurementConsumer(
              VisionMeasurement(selectedPose.ToPose2d(), timestamp, stdDevMat));
        }
      },
      {this});
}

frc::Pose3d Vision::SelectClosestPose(
    const std::array<frc::Pose3d, 2>& fieldSpaceRobotPoses,
    const frc::Pose2d& actualRobotPose) {

  double minDistance = std::numeric_limits<double>::infinity();
  frc::Pose3d selectedPose = frc::Pose3d();

  frc::Pose3d actualRobotPose3d(actualRobotPose);

  for (const auto& pose : fieldSpaceRobotPoses) {
    units::length::meter_t distance = pose.Translation().Distance(actualRobotPose3d.Translation());
    if (double(distance) < minDistance) {
      minDistance = double(distance);
      selectedPose = pose;
    }
  }

  return selectedPose;
}

bool Vision::OutsideFieldBounds(const frc::Pose3d& selectedPose) {
  return selectedPose.Translation().X().value() < 0 ||
         selectedPose.Translation().X().value() > m_filterParameters.aprilTagFieldLayout.GetFieldLength().value() ||
         selectedPose.Translation().Y().value() < 0 ||
         selectedPose.Translation().Y().value() > m_filterParameters.aprilTagFieldLayout.GetFieldWidth().value() ||
         selectedPose.Translation().Z().value() < -m_filterParameters.zMargin.value() ||
         selectedPose.Translation().Z().value() > m_filterParameters.zMargin.value();
}

double Vision::CalculateTagDistance(const std::vector<double>& tagAreas) {
  double largestArea = *std::max_element(tagAreas.begin(), tagAreas.end());

  // Estimate of distance when tag fills up 100 percent of the screen
  double minTagDistance = m_filterParameters.aprilTagWidth /
                          units::length::meter_t(2 * std::tan(double(m_filterParameters.estimatedFOV.Radians() / 2)));

  return (1 / std::sqrt(largestArea / 100.0)) * minTagDistance;
}

Eigen::Vector3d Vision::DetermineStandardDeviation(double tagDistance,
                                                     bool isMultiPose,
                                                     int tagCount) {
  double stdDevFactor = std::pow(tagDistance, 4.0) / tagCount;
  double xyStdDev = m_filterParameters.xyStandardDevBase * stdDevFactor;

  double rotStdDev;
  if (isMultiPose) {
    rotStdDev = m_filterParameters.rotStandardDevBase * stdDevFactor;
  } else {
    rotStdDev = 1e4;
  }

  return Eigen::Vector3d(xyStdDev, xyStdDev, rotStdDev);
}