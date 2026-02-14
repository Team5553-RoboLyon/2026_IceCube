#include "subsystems/turret/turretIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

TurretIOSpark::TurretIOSpark()
{
        // Set the motor configs
    m_motorConfig.SetIdleMode(TurretConstants::motor::IDLE_MODE)
        .Inverted(TurretConstants::motor::INVERTED)
        .SmartCurrentLimit(TurretConstants::motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(TurretConstants::motor::RAMP_RATE)
        .VoltageCompensation(TurretConstants::motor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_motor.Configure(  m_motorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_motor.ClearFaults();
    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(TurretConstants::encoder::DISTANCE_PER_PULSE);
}

void TurretIOSpark::UpdateInputs(TurretIOInputs& inputs) 
{
    inputs.ismotorConnected = (m_motor.GetBusVoltage() !=0.0) && !m_motor.GetFaults().can;

    inputs.motorAppliedVoltage = m_motor.GetAppliedOutput() * TurretConstants::motor::VOLTAGE_COMPENSATION;
    inputs.motorBusVoltage = m_motor.GetBusVoltage();
    inputs.motorCurrent = m_motor.GetOutputCurrent();
    inputs.motorTemperature = m_motor.GetMotorTemperature();
    
    inputs.orientation = m_encoder.GetDistance();
    inputs.hallEffectSensorValue = m_hallEffectSensor.GetVoltage();
}

void TurretIOSpark::SetVoltage(double voltage)
{
        DEBUG_ASSERT((voltage <= TurretConstants::motor::VOLTAGE_COMPENSATION) 
        && (voltage >= -TurretConstants::motor::VOLTAGE_COMPENSATION) 
        ,"Turret Voltage out of range");
    
    m_motor.SetVoltage(units::volt_t(voltage));
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