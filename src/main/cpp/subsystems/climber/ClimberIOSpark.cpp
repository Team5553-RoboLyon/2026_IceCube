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

    inputs.motorBusVoltage = m_climberMotor.GetBusVoltage();
    
    inputs.motorAppliedVoltage = m_climberMotor.GetAppliedOutput() * inputs.motorBusVoltage;
    inputs.motorCurrent = m_climberMotor.GetOutputCurrent();
    inputs.motorTemperature = m_climberMotor.GetMotorTemperature();
    
    inputs.hammerHeight = m_climberEncoder.GetDistance() + ClimberConstants::Settings::BOTTOM_LIMIT;

    inputs.irbreakerValue = m_irbreaker.Get();
    inputs.bottomLimitSwitchValue = m_bottomLimitSwitch.Get();


    #ifdef CLIMBER_SMARTDASHBOARD_LOG
    frc::SmartDashboard::PutBoolean("climber/Motor/isMotorConnected", inputs.isMotorConnected);
    frc::SmartDashboard::PutNumber("climber/Motor/motorAppliedVoltage", inputs.motorAppliedVoltage);
    frc::SmartDashboard::PutNumber("climber/Motor/motorBusVoltage", inputs.motorBusVoltage);
    frc::SmartDashboard::PutNumber("climber/Motor/motorCurrent", inputs.motorCurrent);
    frc::SmartDashboard::PutNumber("climber/Motor/motorTemperature", inputs.motorTemperature);
    frc::SmartDashboard::PutNumber("climber/Sensors/HammerHeight", inputs.hammerHeight);
    frc::SmartDashboard::PutBoolean("climber/Sensors/IRbreakerValue", inputs.irbreakerValue);
    frc::SmartDashboard::PutBoolean("climber/Sensors/bottomLimitSwitchValue", inputs.bottomLimitSwitchValue);
    #else
    m_logger.Log(inputs);
    #endif
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