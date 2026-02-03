#pragma once

#include "rev/SparkMax.h"


#include "ClimberIO.h"
#include "ClimberConstants.h"
#include <frc/Encoder.h>

class ClimberIOSpark  final : public ClimberIO
{
  private:
        rev::spark::SparkMax m_climberMotor { ClimberConstants::climberMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_climberMotorConfig;

    frc::Encoder m_climberEncoder{ClimberConstants::climberEncoder::ID_CHANNEL_A,ClimberConstants::climberEncoder::ID_CHANNEL_B, ClimberConstants::climberEncoder::INVERTED, frc::Encoder::EncodingType::k2X};
  public:
    ClimberIOSpark();
    ~ClimberIOSpark() = default;

    void UpdateInputs(ClimberIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetEncoder() override;    
};