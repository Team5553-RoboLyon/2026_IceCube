#pragma once

#include "rev/SparkMax.h"


#include "climberIO.h"
#include "climberConstants.h"

class ClimberIOSpark  final : public ClimberIO
{
  private:
        rev::spark::SparkMax m_climberMotor { ClimberConstants::climberMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_climberMotorConfig;

  public:
    ClimberIOSpark();
    ~ClimberIOSpark() = default;

    void UpdateInputs(ClimberIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    
};