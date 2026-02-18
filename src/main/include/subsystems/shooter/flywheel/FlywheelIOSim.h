#pragma once

#include "frc/simulation/FlywheelSim.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/system/plant/DCMotor.h"

#include "FlywheelIO.h"
#include "FlywheelConstants.h"

class FlywheelIOSim  final : public FlywheelIO
{
  private:
    frc::DCMotor m_motorsModel{frc::DCMotor::NeoVortex(2)};

    frc::sim::FlywheelSim m_flywheelSim{frc::LinearSystemId::FlywheelSystem(
                                        m_motorsModel,
                                        units::moment_of_inertia::kilogram_square_meter_t(FlywheelConstants::Simulation::MOMENT_OF_INERTIA),
                                        FlywheelConstants::Specifications::GEAR_RATIO), 
                                        m_motorsModel};
  public:
    FlywheelIOSim();
    ~FlywheelIOSim() = default;

    void UpdateInputs(FlywheelIOInputs& inputs) override;
    void SetVoltage(units::volt_t voltage) override;
    void SetDutyCycle(double dutyCycle) override;
    void SetVelocity(units::angular_velocity::revolutions_per_minute_t velocity) override;
};