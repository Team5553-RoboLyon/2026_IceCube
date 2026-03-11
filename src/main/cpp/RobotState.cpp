#include "RobotState.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "LyonLib/utils/MacroUtilsRBL.h"

#include "subsystems/drivetrain/DrivetrainSubsystem.h"
RobotState::RobotState(frc::Pose2d& initialPose,
               frc::DifferentialDriveKinematics& kinematics,
               studica::AHRS& navX)
    :   m_navX(navX),
        m_poseEstimator(
          kinematics,
          initialPose.Rotation(),
          0.0_m,
          0.0_m,
          initialPose,
    #if ROBOT_MODEL != SIMULATION          
          wpi::array<double, 3>{0.2, 0.2, 0.05},     // std dev odometry (m, m, rad)
          wpi::array<double, 3>{0.1, 0.1, 0.3}         // std dev vision (m, m, rad)
    #else
            wpi::array<double, 3>{0.05, 0.05, 0.1},     // std dev odometry (m, m, rad)
            wpi::array<double, 3>{1.0, 1.0, 1.0}         // std dev vision (m, m, rad)
    #endif
      )
  {} 

void RobotState::AddVisionMeasurement(const VisionMeasurement& measurement)
{

    #if ROBOT_MODEL != SIMULATION
  m_poseEstimator.AddVisionMeasurement(
      measurement.robotPose,
      measurement.timestamp,
      measurement.stdDevs
  );

  m_lastVisionPose = measurement.robotPose;
    #endif
}

void RobotState::UpdateOdometry()
{
    #if ROBOT_MODEL != SIMULATION
  frc::Rotation2d heading(units::radian_t(-m_navX.GetYaw() * NF64_PI / 180.0));
  // Détection glissement / bosses
  if (IsOdometryReliable()) {
      m_poseEstimator.UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                                    heading, m_pDrivetrain->GetDistancesSupplier().first, m_pDrivetrain->GetDistancesSupplier().second);
  } else {
      // poids odométrie réduit sur boss
      m_poseEstimator.SetVisionMeasurementStdDevs({0.05, 0.05, 0.1});
      m_poseEstimator.UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                                    heading, m_pDrivetrain->GetDistancesSupplier().first, m_pDrivetrain->GetDistancesSupplier().second);
      // remise à la normale
      m_poseEstimator.SetVisionMeasurementStdDevs({0.1, 0.1, 0.1});
  }
  #else
  m_poseEstimator.UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                                    m_pDrivetrain->GetOdometryPose().Rotation(), m_pDrivetrain->GetDistancesSupplier().first, m_pDrivetrain->GetDistancesSupplier().second);
  #endif
  m_logger.Log(m_poseEstimator.GetEstimatedPosition());
}

std::optional<frc::Pose2d> RobotState::GetPose() const
{
    return m_poseEstimator.GetEstimatedPosition();
}

void RobotState::ResetPoseWithVision()
{
    m_poseEstimator.ResetPosition(frc::Rotation2d(units::radian_t(m_navX.GetYaw() * NF64_PI / 180.0)),
                                  m_pDrivetrain->GetDistancesSupplier().first,
                                  m_pDrivetrain->GetDistancesSupplier().second,
                                  m_lastVisionPose);
    m_pDrivetrain->ResetOdometryPose(m_lastVisionPose);
}

void RobotState::ResetPose(frc::Pose2d &resetPosition)
{
    #if ROBOT_MODEL != SIMULATION
    m_poseEstimator.ResetPosition(frc::Rotation2d(units::radian_t(m_navX.GetYaw() * NF64_PI / 180.0)),
                                  m_pDrivetrain->GetDistancesSupplier().first,
                                  m_pDrivetrain->GetDistancesSupplier().second,
                                  resetPosition);
    #else
                                  m_poseEstimator.ResetPosition(resetPosition.Rotation(),
                                  m_pDrivetrain->GetDistancesSupplier().first,
                                  m_pDrivetrain->GetDistancesSupplier().second,
                                  resetPosition);
    #endif
    m_pDrivetrain->ResetOdometryPose(resetPosition);
}

bool RobotState::IsOdometryReliable() const
{
        constexpr double kMaxPitch = 10.0; // deg
        constexpr double kMaxRoll  = 10.0; // deg
        return (std::abs(m_navX.GetPitch()) < kMaxPitch) &&
               (std::abs(m_navX.GetRoll())  < kMaxRoll);
}