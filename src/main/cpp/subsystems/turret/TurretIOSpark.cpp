#include "subsystems/turret/turretIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

TurretIOSpark::TurretIOSpark()
{
        // Set the motor configs
    m_motorConfig.SetIdleMode(TurretConstants::Motor::IDLE_MODE)
        .Inverted(TurretConstants::Motor::INVERTED)
        .SmartCurrentLimit(TurretConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(TurretConstants::Motor::RAMP_RATE)
        .VoltageCompensation(TurretConstants::Motor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_motor.Configure(  m_motorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_motor.ClearFaults();
    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(TurretConstants::Encoder::DISTANCE_PER_PULSE);
}

void TurretIOSpark::UpdateInputs(TurretIOInputs& inputs) 
{
    inputs.isMotorConnected = (m_motor.GetBusVoltage() !=0.0) && !m_motor.GetFaults().can;

    inputs.motorAppliedVoltage = m_motor.GetAppliedOutput() * TurretConstants::Motor::VOLTAGE_COMPENSATION;
    inputs.motorBusVoltage = m_motor.GetBusVoltage();
    inputs.motorCurrent = m_motor.GetOutputCurrent();
    inputs.motorTemperature = m_motor.GetMotorTemperature();
    
    inputs.orientation = m_encoder.GetDistance() + TurretConstants::Setpoints::INIT_POS;
    inputs.hallEffectSensorValue = m_hallEffectSensor.GetValue();

    #ifndef TURRET_SMARTDASHBOARD_LOG
        m_logger.Log(inputs);
    #else
        frc::SmartDashboard::PutBoolean("turret/Motor/isMotorConnected", inputs.isMotorConnected);
        frc::SmartDashboard::PutNumber("turret/Motor/motorAppliedVoltage", inputs.motorAppliedVoltage);
        frc::SmartDashboard::PutNumber("turret/Motor/motorBusVoltage", inputs.motorBusVoltage);
        frc::SmartDashboard::PutNumber("turret/Motor/motorCurrent", inputs.motorCurrent);
        frc::SmartDashboard::PutNumber("turret/Motor/motorTemperature", inputs.motorTemperature);
        frc::SmartDashboard::PutNumber("turret/Sensors/Orientation", inputs.orientation);
        frc::SmartDashboard::PutNumber("turret/Sensors/HallEffectSensorValue", inputs.hallEffectSensorValue);
    #endif
}

void TurretIOSpark::SetVoltage(units::volt_t voltage)
{
        DEBUG_ASSERT((voltage <= units::volt_t(TurretConstants::Motor::VOLTAGE_COMPENSATION)) 
        && (voltage >= -units::volt_t(TurretConstants::Motor::VOLTAGE_COMPENSATION)) 
        ,"Turret Voltage out of range");
    
    m_motor.SetVoltage(voltage);
}

void TurretIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Turret Duty Cycle out of range");
        m_motor.Set(dutyCycle);
}

void TurretIOSpark::ResetOrientation() 
{
    m_encoder.Reset();
}