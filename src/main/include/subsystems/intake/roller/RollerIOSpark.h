#pragma once

#include "rev/SparkMax.h"
#include <frc/Encoder.h>
#include "frc/DigitalInput.h"

#include "RollerIO.h"
#include "RollerConstants.h"

class RollerIOSpark  final : public RollerIO
{
  private:
    rev::spark::SparkMax m_rollerMotor {RollerConstants::rollerMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_rollerMotorConfig;

  public:
    RollerIOSpark();
    ~RollerIOSpark() = default;

    void UpdateInputs(RollerIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME
};