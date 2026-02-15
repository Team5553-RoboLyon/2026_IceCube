#include "subsystems/shooter/hood/HoodIOSim.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

HoodIOSim::HoodIOSim()
{
    m_hoodSim.SetState({units::radian_t{HoodConstants::Position::MIN}, 0.0});
}

void HoodIOSim::UpdateInputs(HoodIOInputs& inputs) 
{
    m_hoodSim.Update(20_ms);
    inputs.isMotorConnected = true;

    inputs.motorAppliedVoltage = m_hoodSim.GetInput(0);
    inputs.motorBusVoltage = 12.0;
    inputs.motorCurrent = m_hoodSim.GetCurrentDraw().value();
    inputs.motorTemperature = 23.0; 

    inputs.hoodAngle = m_hoodSim.GetAngle().value();

    frc::SmartDashboard::PutNumber("shooter/hood/Motor/AppliedVoltage",inputs.motorAppliedVoltage);
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/BusVoltage", inputs.motorBusVoltage);
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/Current", inputs.motorCurrent);
    frc::SmartDashboard::PutNumber("shooter/hood/Motor/Temperature", inputs.motorTemperature);

    frc::SmartDashboard::PutNumber("shooter/hood/Position", inputs.hoodAngle);

    frc::SmartDashboard::PutBoolean("shooter/hood/Sim/HasHitUpperLimit", m_hoodSim.HasHitUpperLimit());
    frc::SmartDashboard::PutBoolean("shooter/hood/Sim/HasHitLowerLimit", m_hoodSim.HasHitLowerLimit());
}

void HoodIOSim::SetVoltage(units::volt_t voltage)
{
    DEBUG_ASSERT((double(voltage) <= HoodConstants::HoodMotor::VOLTAGE_COMPENSATION) 
        && (double(voltage) >= -HoodConstants::HoodMotor::VOLTAGE_COMPENSATION) 
        ,"Hood voltage out of range");
    
    m_hoodSim.SetInputVoltage(voltage);
}

void HoodIOSim::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Hood Duty Cycle out of range");
    m_hoodSim.SetInputVoltage(units::volt_t{dutyCycle * HoodConstants::HoodMotor::VOLTAGE_COMPENSATION});
}

void HoodIOSim::ResetEncoder()
{
    m_hoodSim.SetState({units::radian_t{HoodConstants::Position::MIN}, 0.0});
}
