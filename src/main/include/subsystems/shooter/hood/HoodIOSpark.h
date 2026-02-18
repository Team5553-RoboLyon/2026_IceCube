#pragma once

#include "rev/SparkMax.h"
#include "frc/Encoder.h"

#include "HoodIO.h"
#include "HoodConstants.h"
#include "HoodLogger.h"

class HoodIOSpark  final : public HoodIO
{
  private:
      rev::spark::SparkMax m_hoodMotor { HoodConstants::HoodMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
      rev::spark::SparkBaseConfig m_hoodMotorConfig;

    frc::Encoder m_hoodEncoder{HoodConstants::HoodEncoder::ID_CHANNEL_A, HoodConstants::HoodEncoder::ID_CHANNEL_B, HoodConstants::HoodEncoder::INVERTED, frc::Encoder::EncodingType::k2X};
    #ifndef HOOD_SMARTDASHBOARD_LOG
    FlywheelIOLogger m_logger{frc::DataLogManager::GetLog(), "/shooter/hood"};
    #endif
  public:
    HoodIOSpark();
    ~HoodIOSpark() = default;

    void UpdateInputs(HoodIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetEncoder() override;
};