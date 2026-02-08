#pragma once

#include "rev/SparkMax.h"
#include <frc/Encoder.h>


#include "intakeIO.h"
#include "intakeConstants.h"

class IntakeIOSpark  final : public IntakeIO
{
  private:
    rev::spark::SparkMax m_intakeMotor { IntakeConstants::intakeMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_intakeMotorConfig;
    rev::spark::SparkMax m_pivotMotor { IntakeConstants::pivotMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_pivotMotorConfig;
    rev::spark::SparkMax m_michelMotor { IntakeConstants::michelMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_michelMotorConfig;

    frc::Encoder m_pivotEncoder{IntakeConstants::Encoder::ID_CHANNEL_A, IntakeConstants::Encoder::ID_CHANNEL_B, IntakeConstants::Encoder::INVERTED, frc::Encoder::EncodingType::k2X};
    
  public:
    IntakeIOSpark();
    ~IntakeIOSpark() = default;

    void UpdateInputs(IntakeIOInputs& inputs) override;
    void SetVoltage(double intakeVoltage, double pivotVoltage, double michelVoltage) override; //COMMENTME
    void SetDutyCycle(double intakeDutyCycle, double pivotDutyCycle, double michelVoltage) override; //COMMENTME

    void ResetEncoder() override;
};