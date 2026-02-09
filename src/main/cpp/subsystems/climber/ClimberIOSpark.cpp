#include "subsystems/climber/climberIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

ClimberIOSpark::ClimberIOSpark()
{
        // Set the motor configs
    m_climberMotorConfig.SetIdleMode(ClimberConstants::Motor::IDLE_MODE)
        .Inverted(ClimberConstants::Motor::INVERTED)
        .SmartCurrentLimit(ClimberConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(ClimberConstants::Motor::RAMP_RATE)
        .VoltageCompensation(ClimberConstants::Motor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_climberMotor.Configure(  m_climberMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_climberMotor.ClearFaults();

    m_climberEncoder.SetDistancePerPulse(ClimberConstants::Encoder::DISTANCE_PER_PULSE);
}

void ClimberIOSpark::UpdateInputs(ClimberIOInputs& inputs) 
{
    inputs.isMotorConnected = (m_climberMotor.GetBusVoltage() !=0.0) && !m_climberMotor.GetFaults().can;

    inputs.motorAppliedVoltage = m_climberMotor.GetAppliedOutput() * ClimberConstants::Motor::VOLTAGE_COMPENSATION;
    inputs.motorBusVoltage = m_climberMotor.GetBusVoltage();
    inputs.motorCurrent = m_climberMotor.GetOutputCurrent();
    inputs.motorTemperature = m_climberMotor.GetMotorTemperature();
    
    inputs.climberHeight = m_climberEncoder.GetDistance();
    inputs.hallEffectSensorValue = m_hallEffectSensor.GetVoltage();
    inputs.bottomLimitSwitchValue = m_bottomLimitSwitch.Get();
}

void ClimberIOSpark::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= ClimberConstants::Motor::VOLTAGE_COMPENSATION) 
        && (voltage >= -ClimberConstants::Motor::VOLTAGE_COMPENSATION) 
        ,"Climber Voltage out of range");
    
    m_climberMotor.SetVoltage(units::volt_t(voltage));
}

void ClimberIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Climber Duty Cycle out of range");
        m_climberMotor.Set(dutyCycle);
}

void ClimberIOSpark::ResetPosition()
{
    m_climberEncoder.Reset();
}