#pragma once

#include "rev/SparkMax.h"

#include "frc/Encoder.h"
#include "frc/DigitalInput.h"

#include "ClimberIO.h"
#include "ClimberConstants.h"
#include "ClimberIOLogger.h"

class ClimberIOSpark  final : public ClimberIO
{
  private:
    rev::spark::SparkMax m_climberMotor { ClimberConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_climberMotorConfig;

    frc::Encoder m_climberEncoder{ClimberConstants::Encoder::ID_CHANNEL_A,ClimberConstants::Encoder::ID_CHANNEL_B, ClimberConstants::Encoder::INVERTED, frc::Encoder::EncodingType::k2X};
    frc::DigitalInput m_bottomLimitSwitch{ClimberConstants::LimitSwitch::BOTTOM_CHANNEL};
    frc::DigitalInput m_irbreaker{ClimberConstants::IRbreaker::CHANNEL};

    #ifndef CLIMBER_SMARTDASHBOARD_LOG
    ClimberIOLogger m_logger{frc::DataLogManager::GetLog(), "/climber"};
    #endif
  public:
    ClimberIOSpark();
    ~ClimberIOSpark() = default;

    void UpdateInputs(ClimberIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetPosition() override;    
};