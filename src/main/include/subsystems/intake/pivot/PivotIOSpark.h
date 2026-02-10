#pragma once

#include "rev/SparkMax.h"
#include <frc/Encoder.h>

#include "PivotIO.h"
#include "PivotConstants.h"

class PivotIOSpark  final : public PivotIO
{
  private:
    rev::spark::SparkMax m_pivotMotor {PivotConstants::pivotMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_pivotMotorConfig;
    rev::spark::SparkClosedLoopController m_pivotMotorController{m_pivotMotor.GetClosedLoopController()};

    frc::Encoder m_rightEncoder{PivotConstants::EncoderRight::ID_CHANNEL_A, PivotConstants::EncoderRight::ID_CHANNEL_B, PivotConstants::EncoderRight::INVERTED, frc::Encoder::EncodingType::k2X};
    frc::Encoder m_leftEncoder{PivotConstants::EncoderLeft::ID_CHANNEL_A, PivotConstants::EncoderLeft::ID_CHANNEL_B, PivotConstants::EncoderLeft::INVERTED, frc::Encoder::EncodingType::k2X};
  public:
    PivotIOSpark();
    ~PivotIOSpark() = default;

    void UpdateInputs(PivotIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME
    void SetTargetPos(double targetPos) override;

    void ResetEncoder() override;
};