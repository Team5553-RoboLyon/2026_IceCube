#pragma once

#include "rev/SparkFlex.h"
#include "frc/Encoder.h"
#include <frc/DigitalInput.h>
#include "frc/Encoder.h"

#include "HoodIO.h"
#include "HoodConstants.h"

class HoodIOSpark  final : public HoodIO
{
  private:
    rev::spark::SparkFlex m_hoodMotor { HoodConstants::HoodMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_hoodMotorConfig;

    frc::Encoder m_hoodEncoder{HoodConstants::HoodEncoder::ID_CHANNEL_A, HoodConstants::HoodEncoder::ID_CHANNEL_B, HoodConstants::HoodEncoder::INVERTED, frc::Encoder::EncodingType::k2X};

  public:
    HoodIOSpark();
    ~HoodIOSpark() = default;

    void UpdateInputs(HoodIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetEncoder() override;
};