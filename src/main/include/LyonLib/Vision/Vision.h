/*******************************************************************************
 * 
 * File        : Vision.h (v1.0)
 * Library     : LyonLib (from 2026)
 * Description : Defines the Vision subsystem, which gathers visual data from 
 *               cameras and processes it to determine the robot's pose.
 * 
 * Authors     : AKA (2026) and inspired by Team 4481
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>


#include <frc/SmartDashboard/SmartDashboard.h>
// #include "networktables/NetworkTableInstance.h"
// #include "networktables/StructArrayTopic.h"
// #include "networktables/StructTopic.h"
// #include "networktables/DoubleTopic.h"

// #include <wpi/math>
// #include <wpi/ArrayRef.h>

#include <array>
#include <functional>
// #include <memory>
#include <vector>

#include "VisionFilterParameters.h"
#include "VisionIO.h"
#include "VisionMeasurement.h"

#include <Eigen/Core>
// #include <frc/MatBuilder.h>
// #include <frc/Numbers.h>

#include "LyonLib/logging/Alert.h"
#include "LyonLib/Vision/VisionIOLogger.h"
#include "LyonLib/logging/ComplexStructLogger.h"

/** A subsystem used to gather visual data using cameras and process it to determine the robot's pose */
class Vision : public frc2::SubsystemBase {
 public:

 /**
     * Creates a new Vision subsystem
     *
     * @param filterParameters The parameters to filter the vision measurements.
     * @param ios The vision IOs representing the different cameras.
     */
  Vision(const VisionFilterParameters& filterParameters,
         std::vector<std::shared_ptr<VisionIO>> ios);

  void Periodic() override;

  /**
     * Processes the vision measurements and adds them to the measurement queue
     *
     * @param robotPoseSupplier A supplier for the robot's pose
     * @param measurementConsumer A consumer for the vision measurements
     * @return A command that processes the vision measurements
     */
  frc2::CommandPtr ProcessVision(
      std::function<frc::Pose2d()> robotPoseSupplier,
      std::function<void(const VisionMeasurement&)> measurementConsumer);

 private:
    /**
     * selects the closest pose to the actual robot pose.
     *
     * @param fieldSpaceRobotPoses The measured poses of the robot in field space.
     * @param actualRobotPose The actual robot pose.
     * @return The closest measured pose to the current robot pose.
     */
  frc::Pose3d SelectClosestPose(const std::array<frc::Pose3d, 2>& fieldSpaceRobotPoses,
                                const frc::Pose2d& actualRobotPose);

    /**
     * Checks if the selected pose is outside the field bounds.
     *
     * @param selectedPose The selected pose.
     * @return True if the selected pose is outside the field bounds, false otherwise.
     */
  bool OutsideFieldBounds(const frc::Pose3d& selectedPose);

  /**
     * Calculates the minimum tag distance based on the tag areas.
     *
     * @param tagAreas The areas of the tags.
     * @return The minimum tag distance.
     */
  double CalculateTagDistance(const std::vector<double>& tagAreas);

  /**
     * Determines the standard deviation of the measurement.
     *
     * @param tagDistance The distance to the tag.
     * @param isMultiPose True if multiple poses are determined for a camera, false otherwise.
     * @param tagCount The number of tags visible.
     * @return The standard deviation of the measurement.
     */
  Eigen::Vector3d DetermineStandardDeviation(double tagDistance,
                                               bool isMultiPose,
                                               int tagCount);

 private:
   VisionFilterParameters m_filterParameters;
  std::vector<std::shared_ptr<VisionIO>> m_visionIOs;
  std::vector<VisionIO::VisionInputs> m_inputs;

  std::vector<VisionIOLogger> m_loggers;
//   std::vector<StructArrayLogger<frc::Pose3d>> m_bestPoseLoggers;
//   std::vector<StructArrayLogger<frc::Pose3d>> m_aternatePoseLoggers;


  std::vector<StructArrayLogger<frc::Pose3d>> m_tagPosesLogger;
  std::vector<Alert> m_connectionAlerts;


  frc::Pose2d m_actualRobotPose = frc::Pose2d();
  std::array<int, 4> m_disconnectCount = {0, 0, 0, 0};
  const int m_maxDisconnectCount = 5;
  const frc::Pose3d m_defaultLogPos = frc::Pose3d(frc::Translation3d(units::meter_t(0),units::meter_t(0),units::meter_t(-5)), 
                                                    frc::Rotation3d());

    // //Logging
    // std::vector<nt::StructPublisher<frc::Pose3d>> m_acceptedPosePubs;
    // std::vector<nt::StructPublisher<frc::Pose3d>> m_tagPosesPubs;
    // std::vector<nt::DoublePublisher> m_distancePubs;
    // std::vector<nt::StructPublisher<Matrix<N3, N1>>> m_stdDevPubs;
};