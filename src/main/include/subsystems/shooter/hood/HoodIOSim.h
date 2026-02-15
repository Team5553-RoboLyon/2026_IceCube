#pragma once

#include "frc/simulation/SingleJointedArmSim.h"
#include "frc/system/plant/DCMotor.h"

#include "HoodIO.h"
#include "HoodConstants.h"
#include "HoodLogger.h"

class HoodIOSim  final : public HoodIO
{
  private:
      frc::DCMotor m_motorModel{frc::DCMotor::NEO550(1)};
      frc::sim::SingleJointedArmSim m_hoodSim{
          m_motorModel,
          HoodConstants::Specifications::GEAR_RATIO,
          units::kilogram_square_meter_t{HoodConstants::Simulation::MOI},
          units::meter_t{HoodConstants::Simulation::ARM_LENGTH},
          units::radian_t{HoodConstants::Position::MIN},
          units::radian_t{HoodConstants::Position::MAX},
          false,
          units::radian_t{HoodConstants::Position::MIN}};
  public:
    HoodIOSim();
    ~HoodIOSim() = default;

    void UpdateInputs(HoodIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetEncoder() override;
};