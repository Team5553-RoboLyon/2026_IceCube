#include "subsystems/intake/roller/RollerIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

RollerIOSpark::RollerIOSpark()
{
        // Set the motor configs
    m_rollerMotorConfig.SetIdleMode(RollerConstants::rollerMotor::IDLE_MODE)
        .Inverted(RollerConstants::rollerMotor::INVERTED)
        .SmartCurrentLimit(RollerConstants::rollerMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(RollerConstants::rollerMotor::RAMP_RATE)
        .VoltageCompensation(RollerConstants::rollerMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_rollerMotor.Configure(m_rollerMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_rollerMotor.ClearFaults();
}

void RollerIOSpark::UpdateInputs(RollerIOInputs& inputs) 
{
    inputs.isRollerMotorConnected = (m_rollerMotor.GetBusVoltage() !=0.0) && !m_rollerMotor.GetFaults().can;

    inputs.rollerMotorBusVoltage = m_rollerMotor.GetBusVoltage();
    inputs.rollerMotorAppliedVoltage = m_rollerMotor.GetAppliedOutput() * inputs.rollerMotorBusVoltage;
    inputs.rollerMotorCurrent = m_rollerMotor.GetOutputCurrent();
    inputs.rollerMotorTemperature = m_rollerMotor.GetMotorTemperature();
    
    #ifdef ROLLER_SMARTDASHBOARD_LOG
    frc::SmartDashboard::PutBoolean("intake/roller/IsMotorConnected", inputs.isRollerMotorConnected);
    frc::SmartDashboard::PutNumber("intake/roller/Motor/AppliedVoltage",inputs.rollerMotorAppliedVoltage);
    frc::SmartDashboard::PutNumber("intake/roller/Motor/BusVoltage", inputs.rollerMotorBusVoltage);
    frc::SmartDashboard::PutNumber("intake/roller/Motor/Current", inputs.rollerMotorCurrent);
    frc::SmartDashboard::PutNumber("intake/roller/MotorTemperature", inputs.rollerMotorTemperature);
    frc::SmartDashboard::PutNumber("intake/roller/MotorVelocity", m_rollerMotor.GetEncoder().GetVelocity()); 
    #else
    m_logger.Log(inputs);
    #endif
}

void RollerIOSpark::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= RollerConstants::rollerMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -RollerConstants::rollerMotor::VOLTAGE_COMPENSATION) 
        ,"Roller Voltage out of range");

    m_rollerMotor.SetVoltage(units::volt_t(voltage));
}

void RollerIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Roller Duty Cycle out of range");
    m_rollerMotor.Set(dutyCycle);
    
}
