#include "subsystems/intake/pivot/PivotIOSim.h"

#include "frc/smartdashboard/SmartDashboard.h"

PivotIOSim::PivotIOSim()
{
    m_pivotSim.SetState(units::radian_t(PivotConstants::Position::HOME_POS),0.0_rad_per_s);
}

void PivotIOSim::UpdateInputs(PivotIOInputs& inputs)
{
    m_pivotSim.Update(20_ms);

    inputs.isLeftEncoderConnected = true;
    inputs.isPivotMotorConnected = true;
    inputs.isRightEncoderConnected = true;

    inputs.pivotMotorAppliedVoltage = m_pivotSim.GetInput(0);
    inputs.pivotMotorBusVoltage = 12.0;
    inputs.pivotMotorCurrent = m_pivotSim.GetCurrentDraw().value();
    inputs.pivotMotorTemperature = 23.0;

    inputs.pivotPos = m_pivotSim.GetAngle().value();

    frc::SmartDashboard::PutBoolean("Pivot motor connected", inputs.isPivotMotorConnected);
    frc::SmartDashboard::PutBoolean("Pivot left encoder connected", inputs.isLeftEncoderConnected);
    frc::SmartDashboard::PutBoolean("Pivot right encoder connected", inputs.isRightEncoderConnected);
    frc::SmartDashboard::PutNumber("Pivot motor applied voltage", inputs.pivotMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("Pivot motor Bus Voltage", inputs.pivotMotorBusVoltage);
    frc::SmartDashboard::PutNumber("Pivot motor current", inputs.pivotMotorCurrent);
    frc::SmartDashboard::PutNumber("Pivot pos", inputs.pivotPos);
}

void PivotIOSim::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= PivotConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -PivotConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        ,"Pivot Voltage out of range");

    m_pivotSim.SetInputVoltage(units::volt_t(voltage));
}

void PivotIOSim::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Intake Duty Cycle out of range");
    m_pivotSim.SetInputVoltage(units::volt_t(dutyCycle*PivotConstants::pivotMotor::VOLTAGE_COMPENSATION));  
}