#pragma once

#include "rev/SparkMax.h"
#include "frc/Encoder.h"

#include "turretIO.h"
#include "turretConstants.h"

class TurretIOSpark  final : public TurretIO
{
  private:
        rev::spark::SparkMax m_motor { TurretConstants::motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_motorConfig;


        frc::Encoder m_encoder {TurretConstants::encoder::A_ID, TurretConstants::encoder::B_ID, TurretConstants::encoder::REVERSED};

  public:
    TurretIOSpark();
    ~TurretIOSpark() = default;

    void UpdateInputs(TurretIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetOrientation() override;
};