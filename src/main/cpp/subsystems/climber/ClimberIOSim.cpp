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
    inputs.motorTemperature = 23.00;
    
    inputs.climberHeight = m_climberSim.GetPosition().value();
    // inputs.hallEffectSensorValue = m_hallEffectSensor.GetVoltage();
    // inputs.bottomLimitSwitchValue = m_bottomLimitSwitch.Get();

    frc::SmartDashboard::PutBoolean("Climber Motor Connected", inputs.isMotorConnected);
    frc::SmartDashboard::PutNumber("Climber Motor Applied Voltage", inputs.motorAppliedVoltage);
    frc::SmartDashboard::PutNumber("Climber Motor Bus Voltage", inputs.motorBusVoltage);
    frc::SmartDashboard::PutNumber("Climber Motor Current", inputs.motorCurrent);
    frc::SmartDashboard::PutNumber("Climber Motor Temperature", inputs.motorTemperature);

    frc::SmartDashboard::PutNumber("Climber Sim Height", inputs.climberHeight);
    frc::SmartDashboard::PutBoolean("Has Hit Lower Limit", m_climberSim.HasHitLowerLimit());
    frc::SmartDashboard::PutBoolean("Has Hit Upper Limit", m_climberSim.HasHitUpperLimit());
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
    // m_climberEncoder.Reset();
}