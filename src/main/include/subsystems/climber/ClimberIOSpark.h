#pragma once

#include "rev/SparkMax.h"

#include "frc/Encoder.h"
#include "frc/DigitalInput.h"
#include "frc/AnalogInput.h"

#include "ClimberIO.h"
#include "ClimberConstants.h"

class ClimberIOSpark  final : public ClimberIO
{
  private:
        rev::spark::SparkMax m_climberMotor { ClimberConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_climberMotorConfig;

    frc::Encoder m_climberEncoder{ClimberConstants::Encoder::ID_CHANNEL_A,ClimberConstants::Encoder::ID_CHANNEL_B, ClimberConstants::Encoder::INVERTED, frc::Encoder::EncodingType::k2X};
    frc::DigitalInput m_bottomLimitSwitch{ClimberConstants::LimitSwitch::BOTTOM_CHANNEL};
    frc::AnalogInput m_hallEffectSensor{ClimberConstants::HallEffectSensor::CHANNEL};
  public:
    ClimberIOSpark();
    ~ClimberIOSpark() = default;

    void UpdateInputs(ClimberIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetPosition() override;    
};