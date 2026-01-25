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

class Vision : public frc2::SubsystemBase {
 public:
  Vision(const VisionFilterParameters& filterParameters,
         std::vector<std::shared_ptr<VisionIO>> ios);

  void Periodic() override;

  frc2::CommandPtr ProcessVision(
      std::function<frc::Pose2d()> robotPoseSupplier,
      std::function<void(const VisionMeasurement&)> measurementConsumer);

 private:
  frc::Pose3d SelectClosestPose(const std::array<frc::Pose3d, 2>& fieldSpaceRobotPoses,
                                const frc::Pose2d& actualRobotPose);

  bool OutsideFieldBounds(const frc::Pose3d& selectedPose);

  double CalculateTagDistance(const std::vector<double>& tagAreas);

  Eigen::Vector3d DetermineStandardDeviation(double tagDistance,
                                               bool isMultiPose,
                                               int tagCount);

 private:
  std::vector<std::shared_ptr<VisionIO>> m_visionIOs;
  std::vector<VisionIO::VisionInputs> m_inputs;
  std::vector<Alert> m_connectionAlerts;
  VisionFilterParameters m_filterParameters;

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