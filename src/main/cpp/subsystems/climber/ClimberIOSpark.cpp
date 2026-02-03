#include "subsystems/climber/climberIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

ClimberIOSpark::ClimberIOSpark()
{
        // Set the motor configs
    m_climberMotorConfig.SetIdleMode(ClimberConstants::climberMotor::IDLE_MODE)
        .Inverted(ClimberConstants::climberMotor::INVERTED)
        .SmartCurrentLimit(ClimberConstants::climberMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(ClimberConstants::climberMotor::RAMP_RATE)
        .VoltageCompensation(ClimberConstants::climberMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_climberMotor.Configure(  m_climberMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_climberMotor.ClearFaults();

    m_climberEncoder.SetDistancePerPulse(ClimberConstants::climberEncoder::DISTANCE_PER_PULSE);
    
}

void ClimberIOSpark::UpdateInputs(ClimberIOInputs& inputs) 
{
    inputs.isclimberMotorConnected = (m_climberMotor.GetBusVoltage() !=0.0) && !m_climberMotor.GetFaults().can;

    inputs.climberMotorAppliedVoltage = m_climberMotor.GetAppliedOutput() * ClimberConstants::climberMotor::VOLTAGE_COMPENSATION;
    inputs.climberMotorBusVoltage = m_climberMotor.GetBusVoltage();
    inputs.climberMotorCurrent = m_climberMotor.GetOutputCurrent();
    inputs.climberMotorTemperature = m_climberMotor.GetMotorTemperature();
    
    inputs.climberPos = m_climberEncoder.GetDistance();
}

void ClimberIOSpark::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= ClimberConstants::climberMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -ClimberConstants::climberMotor::VOLTAGE_COMPENSATION) 
        ,"Climber Voltage out of range");
    
    m_climberMotor.SetVoltage(units::volt_t(voltage));
}

void ClimberIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Climber Duty Cycle out of range");
        m_climberMotor.Set(dutyCycle);
}

void ClimberIOSpark::ResetEncoder()
{
    m_climberEncoder.Reset();
}