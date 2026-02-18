#pragma once

#include "turretIO.h"
#include "turretConstants.h"

#include "frc/system/plant/DCMotor.h"
#include "frc/simulation/SingleJointedArmSim.h"

class TurretIOSim  final : public TurretIO
{
  private:
    frc::DCMotor m_motorModel{frc::DCMotor::NEO550(1)};

    frc::sim::SingleJointedArmSim m_simTurret{
      m_motorModel, 
      TurretConstants::Specifications::GEAR_RATIO,
      units::kilogram_square_meter_t(TurretConstants::Simulation::MOI),
      units::meter_t(TurretConstants::Simulation::RADIUS), //useless for a turret, but required by the constructor
      units::radian_t(TurretConstants::Settings::BOTTOM_LIMIT),
      units::radian_t(TurretConstants::Settings::TOP_LIMIT),
      false,
      units::radian_t(0.0)};

  public:
    TurretIOSim();
    ~TurretIOSim() = default;

    void UpdateInputs(TurretIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override;
    void SetDutyCycle(double dutyCycle) override;

    void ResetOrientation() override;
};