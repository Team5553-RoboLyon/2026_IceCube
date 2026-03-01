#pragma once

#include "rev/SparkFlex.h"

#include "FlywheelIO.h"
#include "FlywheelConstants.h"
#include "FlywheelLogger.h"

class FlywheelIOSpark  final : public FlywheelIO
{
  private:
    rev::spark::SparkFlex m_leftMotor {FlywheelConstants::LeftMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_leftMotorConfig;
    rev::spark::SparkFlex m_rightMotor {FlywheelConstants::RightMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_rightMotorConfig;

    #ifndef FLYWHEEL_SMARTDASHBOARD_LOG
    FlywheelIOLogger m_logger{frc::DataLogManager::GetLog(), "/shooter/flywheel"};
    #endif
  public:
    FlywheelIOSpark();
    ~FlywheelIOSpark() = default;

    void UpdateInputs(FlywheelIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override;
    void SetDutyCycle(double dutyCycle) override;
    void SetVelocity(units::angular_velocity::revolutions_per_minute_t velocity) override;
};