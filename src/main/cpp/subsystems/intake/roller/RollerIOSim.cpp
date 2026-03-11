#include "subsystems/intake/roller/RollerIOSim.h"

#include "frc/smartdashboard/Smartdashboard.h"

RollerIOSim::RollerIOSim()
{
}

void RollerIOSim::UpdateInputs(RollerIOInputs& inputs)
{
    m_rollerSim.Update(20_ms);

    inputs.isRollerMotorConnected = true;

    inputs.rollerMotorAppliedVoltage = m_rollerSim.GetInput(0);
    inputs.rollerMotorBusVoltage = 12.0;
    inputs.rollerMotorCurrent = m_rollerSim.GetCurrentDraw().value();
    inputs.rollerMotorTemperature = 23.0;
    
    frc::SmartDashboard::PutBoolean("intake/roller/IsMotorConnected", inputs.isRollerMotorConnected);
    frc::SmartDashboard::PutNumber("intake/roller/Motor/AppliedVoltage",inputs.rollerMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("intake/roller/Motor/BusVoltage", inputs.rollerMotorBusVoltage);
    frc::SmartDashboard::PutNumber("intake/roller/Motor/Current", inputs.rollerMotorCurrent);
    frc::SmartDashboard::PutNumber("intake/roller/MotorTemperature", inputs.rollerMotorTemperature);
    frc::SmartDashboard::PutNumber("intake/roller/MotorVelocity", m_rollerSim.GetAngularVelocity().value()); 
}

void RollerIOSim::SetVoltage(double voltage)
{
    m_rollerSim.SetInputVoltage(units::volt_t(voltage));
}

void RollerIOSim::SetDutyCycle(double dutyCycle)
{
    m_rollerSim.SetInputVoltage(units::volt_t(dutyCycle*RollerConstants::rollerMotor::VOLTAGE_COMPENSATION));
}
