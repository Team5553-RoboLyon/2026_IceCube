#pragma once

#include "DrivetrainIO.h"
#include "DrivetrainConstants.h"

#include "frc/simulation/DifferentialDrivetrainSim.h"
#include "frc/system/plant/DCMotor.h"

// #include "LyonLib/localization/TankOdometryTracker.h"
#include "LyonLib/logging/ComplexStructLogger.h"

class DrivetrainIOSim  final : public DrivetrainIO
{
  private:
  frc::DCMotor m_motorsPerSideModel{frc::DCMotor::NeoVortex(2)};
  frc::sim::DifferentialDrivetrainSim m_drivetrainSim{
    m_motorsPerSideModel, 
    driveConstants::Specifications::GEAR_RATIO, 
    units::kilogram_square_meter_t{driveConstants::Simulation::MOI},
    units::kilogram_t{driveConstants::Simulation::MASS},
    units::meter_t{driveConstants::Specifications::WHEEL_RADIUS}, 
    units::meter_t{driveConstants::Specifications::TRACKWIDTH}
  };

  // double m_realLeftSideSpeed{0.0};
  // double m_realRightSideSpeed{0.0};
  // TankOdometryTracker m_odometry{&m_realLeftSideSpeed, &m_realRightSideSpeed, driveConstants::Specifications::TRACKWIDTH};

  units::volt_t m_leftVoltage{0_V};
  units::volt_t m_rightVoltage{0_V};
  StructLogger<frc::Pose2d> robotPoseLogger{"Drivetrain/Odometry/Pose2d"};

  public:
    DrivetrainIOSim();
    ~DrivetrainIOSim() = default;

    void UpdateInputs(DrivetrainIOInputs& inputs) override;

    void SetVoltage(const units::volt_t leftSideVoltage, const units::volt_t rightSideVoltage) override;
    void SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle) override;
    void SetChassisSpeed(const frc::ChassisSpeeds &speeds) override;
    
    void ResetPosition(const frc::Pose2d position) override;
};