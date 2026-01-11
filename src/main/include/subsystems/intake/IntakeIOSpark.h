#pragma once

#include "rev/SparkMax.h"


#include "intakeIO.h"
#include "intakeConstants.h"

class IntakeIOSpark  final : public IntakeIO
{
  private:
        rev::spark::SparkMax m_leftMotor { IntakeConstants::leftMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_leftMotorConfig;
    rev::spark::SparkMax m_rightMotor { IntakeConstants::rightMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_rightMotorConfig;


    

  public:
    IntakeIOSpark();
    ~IntakeIOSpark() = default;

    void UpdateInputs(IntakeIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    
};