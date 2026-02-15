#pragma once

#include "rev/SparkMax.h"

#include "frc/Encoder.h"
#include "frc/DigitalInput.h"
#include "frc/AnalogInput.h"

#include "frc/simulation/ElevatorSim.h"
#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/plant/DCMotor.h"

#include "ClimberIO.h"
#include "ClimberConstants.h"

class ClimberIOSim  final : public ClimberIO
{
  private:
    frc::DCMotor m_motorModel{frc::DCMotor::NEO(1)};

    frc::sim::ElevatorSim m_climberSim{m_motorModel, 
                                      ClimberConstants::Specifications::GEAR_RATIO, 
                                      units::kilogram_t{ClimberConstants::Simulation::CARRIAGE_MASSE}, 
                                      units::meter_t{ClimberConstants::Specifications::DRUM_RADIUS},
                                      units::meter_t{ClimberConstants::Settings::BOTTOM_LIMIT}, 
                                      units::meter_t{ClimberConstants::Settings::TOP_LIMIT},
                                      true,
                                      units::meter_t{ClimberConstants::Settings::BOTTOM_LIMIT}};

    // frc::DigitalInput m_bottomLimitSwitch{ClimberConstants::LimitSwitch::BOTTOM_CHANNEL};
    // frc::AnalogInput m_hallEffectSensor{ClimberConstants::HallEffectSensor::CHANNEL};
  public:
    ClimberIOSim();
    ~ClimberIOSim() = default;

    void UpdateInputs(ClimberIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    void ResetPosition() override;    
};