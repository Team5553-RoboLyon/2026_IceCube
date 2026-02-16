#include "subsystems/turret/TurretIOSim.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

TurretIOSim::TurretIOSim()
{
     m_simTurret.SetState(units::radian_t(0.0), units::radians_per_second_t(0.0));
}

void TurretIOSim::UpdateInputs(TurretIOInputs& inputs) 
{
    m_simTurret.Update(20_ms);
    inputs.isMotorConnected = true;

    inputs.motorAppliedVoltage = m_simTurret.GetInput(0);
    inputs.motorBusVoltage = 12.0;
    inputs.motorCurrent = m_simTurret.GetCurrentDraw().value();
    inputs.motorTemperature = 23.0; 
    
    inputs.orientation = m_simTurret.GetAngle().value();
    inputs.hallEffectSensorValue = 0.0; //TODO : Simulated hall effect sensor not implemented
    frc::SmartDashboard::PutBoolean("turret/Motor/isMotorConnected", inputs.isMotorConnected);
    frc::SmartDashboard::PutNumber("turret/Motor/motorAppliedVoltage", inputs.motorAppliedVoltage);
    frc::SmartDashboard::PutNumber("turret/Motor/motorBusVoltage", inputs.motorBusVoltage);
    frc::SmartDashboard::PutNumber("turret/Motor/motorCurrent", inputs.motorCurrent);
    frc::SmartDashboard::PutNumber("turret/Motor/motorTemperature", inputs.motorTemperature);
    frc::SmartDashboard::PutNumber("turret/Sensors/Orientation", inputs.orientation);
    frc::SmartDashboard::PutNumber("turret/Sensors/HallEffectSensorValue", inputs.hallEffectSensorValue);

    frc::SmartDashboard::PutBoolean("turret/Sim/HasHitLowerLimit", m_simTurret.HasHitLowerLimit());
    frc::SmartDashboard::PutBoolean("turret/Sim/HasHitUpperLimit", m_simTurret.HasHitUpperLimit());
}

void TurretIOSim::SetVoltage(units::volt_t voltage)
{
        DEBUG_ASSERT((voltage <= units::volt_t(TurretConstants::Motor::VOLTAGE_COMPENSATION)) 
        && (voltage >= -units::volt_t(TurretConstants::Motor::VOLTAGE_COMPENSATION)) 
        ,"Turret Voltage out of range");
    
    m_simTurret.SetInputVoltage(voltage);
}

void TurretIOSim::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Turret Duty Cycle out of range");
        m_simTurret.SetInputVoltage(units::volt_t(dutyCycle * TurretConstants::Motor::VOLTAGE_COMPENSATION));
}

void TurretIOSim::ResetOrientation() 
{
    m_simTurret.SetState(units::radian_t(0.0), units::radians_per_second_t(0.0));
}