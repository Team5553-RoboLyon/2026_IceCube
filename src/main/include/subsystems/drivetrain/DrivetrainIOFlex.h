#pragma once

#include "rev/SparkFlex.h"
#include "frc/Encoder.h"

#include "DrivetrainIO.h"
#include "DrivetrainConstants.h"
#include "DrivetrainIOLogger.h"


#include "LyonLib/localization/TankOdometryTracker.h"
#include "LyonLib/logging/ComplexStructLogger.h"

class DrivetrainIOFlex  final : public DrivetrainIO
{
  private:
  rev::spark::SparkFlex m_motorFrontLeft{driveConstants::Motors::LEFT_FRONT_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorBackLeft{driveConstants::Motors::LEFT_BACK_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorFrontRight{driveConstants::Motors::RIGHT_FRONT_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorBackRight{driveConstants::Motors::RIGHT_BACK_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};

  rev::spark::SparkBaseConfig m_motorFrontLeftConfig{};
  rev::spark::SparkBaseConfig m_motorBackLeftConfig{};
  rev::spark::SparkBaseConfig m_motorFrontRightConfig{};
  rev::spark::SparkBaseConfig m_motorBackRightConfig{};

  frc::Encoder m_encoderLeft{driveConstants::Encoder::LEFT_ID_ENCODER_A, 
                            driveConstants::Encoder::LEFT_ID_ENCODER_B, 
                            driveConstants::Encoder::LEFT_REVERSE_ENCODER};
  frc::Encoder m_encoderRight{driveConstants::Encoder::RIGHT_ID_ENCODER_A,
                            driveConstants::Encoder::RIGHT_ID_ENCODER_B,
                            driveConstants::Encoder::RIGHT_REVERSE_ENCODER};

  double m_realLeftSideSpeed{0.0};
  double m_realRightSideSpeed{0.0};

  TankOdometryTracker m_odometry{&m_realLeftSideSpeed, &m_realRightSideSpeed, driveConstants::Specifications::TRACKWIDTH};
  #ifndef DRIVETRAIN_SMARTDASHBOARD_LOG
  DrivetrainIOLogger m_logger{frc::DataLogManager::GetLog(), "Drivetrain"};
  #endif
  StructLogger<frc::Pose2d> robotPoseLogger{"Drivetrain/Odometry/Pose2d"};

  public:
    DrivetrainIOFlex();
    ~DrivetrainIOFlex() = default;

    void UpdateInputs(DrivetrainIOInputs& inputs) override;

    void SetVoltage(const units::volt_t leftSideVoltage, const units::volt_t rightSideVoltage) override;
    void SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle) override;
    void SetChassisSpeed(const frc::ChassisSpeeds &speeds) override;
    
    void ResetPosition(const frc::Pose2d position) override;
};