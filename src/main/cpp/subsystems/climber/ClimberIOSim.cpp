#include "subsystems/climber/ClimberIOSim.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

ClimberIOSim::ClimberIOSim()
{
    m_climberSim.SetState({units::meter_t{ClimberConstants::Settings::BOTTOM_LIMIT}, 0.0});
}

void ClimberIOSim::UpdateInputs(ClimberIOInputs& inputs) 
{
    m_climberSim.Update(units::second_t(20_ms)); //update the sim with a 20ms timestep
    inputs.isMotorConnected = true;
    inputs.motorAppliedVoltage = m_climberSim.GetInput(0);
    inputs.motorBusVoltage = 12.0;
    inputs.motorCurrent = m_climberSim.GetCurrentDraw().value();
    inputs.motorTemperature = 23.0;
    
    inputs.hammerHeight = m_climberSim.GetPosition().value();
    inputs.irbreakerValue = m_climberSim.GetPosition().value() <= ClimberConstants::Settings::IRBREAKER_TRIGGER_HEIGHT;
    inputs.bottomLimitSwitchValue = m_climberSim.HasHitLowerLimit();

    frc::SmartDashboard::PutBoolean("climber/Motor/isMotorConnected", inputs.isMotorConnected);
    frc::SmartDashboard::PutNumber("climber/Motor/motorAppliedVoltage", inputs.motorAppliedVoltage);
    frc::SmartDashboard::PutNumber("climber/Motor/motorBusVoltage", inputs.motorBusVoltage);
    frc::SmartDashboard::PutNumber("climber/Motor/motorCurrent", inputs.motorCurrent);
    frc::SmartDashboard::PutNumber("climber/Motor/motorTemperature", inputs.motorTemperature);
    frc::SmartDashboard::PutNumber("climber/Sensors/HammerHeight", inputs.hammerHeight);
    frc::SmartDashboard::PutBoolean("climber/Sensors/IRbreakerValue", inputs.irbreakerValue);
    frc::SmartDashboard::PutBoolean("climber/Sensors/bottomLimitSwitchValue", inputs.bottomLimitSwitchValue);


    frc::SmartDashboard::PutBoolean("climber/Sim/HasHitUpperLimit", m_climberSim.HasHitUpperLimit());
}

void ClimberIOSim::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= ClimberConstants::Motor::VOLTAGE_COMPENSATION) 
        && (voltage >= -ClimberConstants::Motor::VOLTAGE_COMPENSATION) 
        ,"Climber Voltage out of range");
    
    m_climberSim.SetInputVoltage(units::volt_t(voltage));
}

void ClimberIOSim::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Climber Duty Cycle out of range");
    m_climberSim.SetInputVoltage(units::volt_t(dutyCycle * ClimberConstants::Motor::VOLTAGE_COMPENSATION));
}

void ClimberIOSim::ResetPosition()
{
    m_climberSim.SetState({units::meter_t{ClimberConstants::Settings::BOTTOM_LIMIT}, 0.0});
}