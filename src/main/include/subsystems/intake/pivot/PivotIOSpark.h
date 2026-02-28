#pragma once

#include "rev/SparkMax.h"
#include <frc/DutyCycleEncoder.h>

#include "PivotIO.h"
#include "PivotConstants.h"

class PivotIOSpark  final : public PivotIO
{
  private:
    rev::spark::SparkMax m_pivotMotor {PivotConstants::pivotMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_pivotMotorConfig;
    rev::spark::SparkClosedLoopController m_pivotMotorController{m_pivotMotor.GetClosedLoopController()};

    frc::DutyCycleEncoder m_rightEncoder{PivotConstants::EncoderRight::ID, PivotConstants::EncoderRight::FULL_RANGE, PivotConstants::EncoderRight::EXPECTED_ZERO};
    frc::DutyCycleEncoder m_leftEncoder{PivotConstants::EncoderLeft::ID, PivotConstants::EncoderLeft::FULL_RANGE, PivotConstants::EncoderLeft::EXPECTED_ZERO};

  public:
    PivotIOSpark();
    ~PivotIOSpark() = default;

    void UpdateInputs(PivotIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME
};