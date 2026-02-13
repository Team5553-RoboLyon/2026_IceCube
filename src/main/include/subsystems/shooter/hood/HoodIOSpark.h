#pragma once

#include "rev/SparkFlex.h"
#include "frc/Encoder.h"
#include <frc/DigitalInput.h>

#include "HoodIO.h"
#include "HoodConstants.h"

class HoodIOSpark  final : public HoodIO
{
  private:
    rev::spark::SparkFlex m_hoodMotor { HoodConstants::HoodMotor::ID, rev::spark::SparkFlex::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_hoodMotorConfig;

  public:
    HoodIOSpark();
    ~HoodIOSpark() = default;

    void UpdateInputs(HoodIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME
};