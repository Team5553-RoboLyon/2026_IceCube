#pragma once

#include "rev/SparkMax.h"
#include "frc/Encoder.h"
#include "frc/AnalogInput.h"

#include "turretIO.h"
#include "turretConstants.h"
#include "turretIOLogger.h"

class TurretIOSpark  final : public TurretIO
{
  private:
    rev::spark::SparkMax m_motor{TurretConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_motorConfig;
    
    frc::Encoder m_encoder{TurretConstants::Encoder::A_ID, TurretConstants::Encoder::B_ID, TurretConstants::Encoder::REVERSED};

    frc::AnalogInput m_hallEffectSensor{TurretConstants::HallEffectSensor::ID};

    #ifndef TURRET_SMARTDASHBOARD_LOG
      TurretIOLogger m_logger{frc::DataLogManager::GetLog(), "/turret"};
    #endif
  public:
    TurretIOSpark();
    ~TurretIOSpark() = default;

    void UpdateInputs(TurretIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override;
    void SetDutyCycle(double dutyCycle) override;

    void ResetOrientation() override;
};