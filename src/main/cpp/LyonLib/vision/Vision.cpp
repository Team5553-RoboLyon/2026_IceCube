#include "Lyonlib/vision/Vision.h"

Vision::Vision(const VisionFilterParameters& filterParameters,
               std::vector<std::shared_ptr<VisionIO>> ios)
    : m_filterParameters(filterParameters),
      m_visionIOs(std::move(ios)) {

  m_inputs.resize(m_visionIOs.size());
  m_connectionAlerts.reserve(m_visionIOs.size());
  m_loggers.reserve(m_visionIOs.size());
  // m_bestPoseLoggers.reserve(m_visionIOs.size());
  // m_aternatePoseLoggers.reserve(m_visionIOs.size());
  m_tagPosesLogger.reserve(m_visionIOs.size());


  for (size_t i = 0; i < m_visionIOs.size(); ++i) {
    m_connectionAlerts.emplace_back(
        "Alerts/Vision/" + std::to_string(i),
        "[Camera " + std::to_string(i) + "] disconnected",
        Alert::AlertType::ERROR);
    m_loggers.emplace_back(frc::DataLogManager::GetLog(), m_inputs[i].cameraName, "Vision");
    m_tagPosesLogger.emplace_back("Vision/" + m_inputs[i].cameraName + "/Tag Poses");
  }
}


void Vision::Periodic() {
  // Update each io, which represents the different cameras
  for (size_t i = 0; i < m_visionIOs.size(); ++i) {
    m_visionIOs[i]->UpdateInputs(m_inputs[i]);
    m_visionIOs[i]->SetRobotRotation(m_actualRobotPose.Rotation());

    // Error handling for the vision IO
    m_connectionAlerts[i].Set(!m_inputs[i].connected);
    frc::SmartDashboard::PutBoolean(
        "Connection Status/Camera " + m_inputs[i].cameraName,
        m_inputs[i].connected);

    m_loggers[i].Log(m_inputs[i]);

    // frc::SmartDashboard::PutBoolean("Vision/" + m_inputs[i].cameraName + "/Has Results",
    //                                m_inputs[i].hasResults);
    // frc::SmartDashboard::PutBoolean("Vision/" + m_inputs[i].cameraName + "/Is Connected",
    //                                m_inputs[i].connected);
    // frc::SmartDashboard::PutNumber("Vision/" + m_inputs[i].cameraName + "/Ambiguity Ratio",
    //                               m_inputs[i].ambiguityRatio);

    // // Convert integer tag IDs to double for PutNumberArray which expects numeric doubles
    // std::vector<double> visibleTagIDsDouble;
    // visibleTagIDsDouble.reserve(m_inputs[i].visibleTagIDs.size());
    // for (auto id : m_inputs[i].visibleTagIDs) {
    //   visibleTagIDsDouble.push_back(static_cast<double>(id));
    // }
    // frc::SmartDashboard::PutNumberArray("Vision/" + m_inputs[i].cameraName + "/Visible Tag IDs",
    //                                     visibleTagIDsDouble);

    // // m_PoseLoggers[i].Log(m_inputs[i].fieldSpaceRobotPoses[0]);
    // //TODO : log in smartdashboard
        
    
    // frc::SmartDashboard::PutNumber("Vision/" + m_inputs[i].cameraName + "/Tag Area",
    //                               m_inputs[i].tagAreas.empty() ? 0.0 : *std::max_element(m_inputs[i].tagAreas.begin(), m_inputs[i].tagAreas.end()));
    // frc::SmartDashboard::PutNumber("Vision/" + m_inputs[i].cameraName + "/Visible Tag IDs",
    //                               m_inputs[i].visibleTagIDs.size());
  }
}

frc2::CommandPtr Vision::ProcessVision(
    std::function<frc::Pose2d()> robotPoseSupplier,
    std::function<void(const VisionMeasurement&)> measurementConsumer) {

  return frc2::cmd::Run(
      [this, robotPoseSupplier, measurementConsumer] {
        m_actualRobotPose = robotPoseSupplier();

         // Cycle through all cameras
        for (size_t i = 0; i < m_visionIOs.size(); ++i) {
          if (!m_inputs[i].connected || !m_inputs[i].hasResults) {
            m_disconnectCount[i]++;

            if (m_disconnectCount[i] > m_maxDisconnectCount) {
            //TODO : log in dashboard default pose
                
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
          m_tagPosesLogger[i].Log(tagPoses);

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

          // Determine the standard deviation of the measurement using the distance
          // The distance is estimated based on the tag area in percent (0-100)
          double tagDistance = CalculateTagDistance(m_inputs[i].tagAreas);
          frc::SmartDashboard::PutNumber("Vision/" + m_inputs[i].cameraName + "/Estimated Tag Distance", tagDistance);

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

        // Log the accepted pose
        frc::SmartDashboard::PutNumberArray("Vision/" + m_inputs[i].cameraName + "/stdDevMat",
                                      std::array<double, 3>{stdDevMat.x(), stdDevMat.y(), stdDevMat.z()});
          measurementConsumer(
              VisionMeasurement(selectedPose.ToPose2d(), timestamp, stdDevMat));
        }
      },
      {this});
}

frc::Pose3d Vision::SelectClosestPose(
    const std::array<frc::Pose3d, 2>& fieldSpaceRobotPoses,
    const frc::Pose2d& actualRobotPose) {

  // Get the distance closest to the current robot pose if multiple poses are determined for a
  // camera
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
  // Get the largest area, this will be the closest tag
  double largestArea = *std::max_element(tagAreas.begin(), tagAreas.end());

  // Estimate of distance when tag fills up 100 percent of the screen
  double minTagDistance = m_filterParameters.aprilTagWidth /
                          units::length::meter_t(2 * std::tan(double(m_filterParameters.estimatedFOV.Radians() / 2)));

  // Estimate the tag distance in meters
  return (1 / std::sqrt(largestArea / 100.0)) * minTagDistance;
}

Eigen::Vector3d Vision::DetermineStandardDeviation(double tagDistance,
                                                     bool isMultiPose,
                                                     int tagCount) {
  // Determine the standard deviation of the measurement
  double stdDevFactor = std::pow(tagDistance, 4.0) / tagCount;
  double xyStdDev = m_filterParameters.xyStandardDevBase * stdDevFactor;

  double rotStdDev;
  if (isMultiPose) {
    // Multi tag result
    rotStdDev = m_filterParameters.rotStandardDevBase * stdDevFactor;
  } else {
    // Single tag result
    // Determine the corresponding standard deviation for this result
    // When the distance is 1 meter, the standard deviation is the base value
    rotStdDev = 1e4; // Don't use rotation result when only one tag is visible
  }

  return Eigen::Vector3d(xyStdDev, xyStdDev, rotStdDev);
}