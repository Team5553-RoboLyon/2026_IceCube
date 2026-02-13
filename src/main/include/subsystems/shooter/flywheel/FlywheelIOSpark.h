#pragma once

#include "rev/SparkFlex.h"
#include "frc/Encoder.h"
#include <frc/DigitalInput.h>

#include "FlywheelIO.h"
#include "FlywheelConstants.h"

class FlywheelIOSpark  final : public FlywheelIO
{
  private:
    rev::spark::SparkFlex m_leftMotor {FlywheelConstants::LeftMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_leftMotorConfig;
    rev::spark::SparkFlex m_rightMotor {FlywheelConstants::RightMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_rightMotorConfig;

  public:
    FlywheelIOSpark();
    ~FlywheelIOSpark() = default;

    void UpdateInputs(FlywheelIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME
};