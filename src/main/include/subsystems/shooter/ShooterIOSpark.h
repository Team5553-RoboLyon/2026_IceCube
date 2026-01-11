#pragma once

#include "rev/SparkFlex.h"
#include "frc/Encoder.h"
#include <frc/DigitalInput.h>

#include "shooterIO.h"
#include "shooterConstants.h"

class ShooterIOSpark  final : public ShooterIO
{
  private:
    rev::spark::SparkFlex m_LeftMotor { ShooterConstants::LeftMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_LeftMotorConfig;
    rev::spark::SparkFlex m_RightMotor { ShooterConstants::RightMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_RightMotorConfig;


    frc::Encoder m_WheelEncoder {ShooterConstants::WheelEncoder::A_ID, ShooterConstants::WheelEncoder::B_ID, ShooterConstants::WheelEncoder::REVERSED};
    frc::DigitalInput m_IRBreakerOutput {ShooterConstants::IRBreakerOutput::ID};

  public:
    ShooterIOSpark();
    ~ShooterIOSpark() = default;

    void UpdateInputs(ShooterIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetRotation() override;

};