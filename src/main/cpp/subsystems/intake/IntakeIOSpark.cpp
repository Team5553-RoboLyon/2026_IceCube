#include "subsystems/intake/intakeIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

IntakeIOSpark::IntakeIOSpark()
{
        // Set the motor configs
    m_intakeMotorConfig.SetIdleMode(IntakeConstants::intakeMotor::IDLE_MODE)
        .Inverted(IntakeConstants::intakeMotor::INVERTED)
        .SmartCurrentLimit(IntakeConstants::intakeMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(IntakeConstants::intakeMotor::RAMP_RATE)
        .VoltageCompensation(IntakeConstants::intakeMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_intakeMotor.Configure(  m_intakeMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_intakeMotor.ClearFaults();
    // Set the motor configs
    m_pivotMotorConfig.SetIdleMode(IntakeConstants::pivotMotor::IDLE_MODE)
        .Inverted(IntakeConstants::pivotMotor::INVERTED)
        .SmartCurrentLimit(IntakeConstants::pivotMotor::CURRENT_LIMIT)
        .ClosedLoopRampRate(IntakeConstants::pivotMotor::RAMP_RATE)
        .VoltageCompensation(IntakeConstants::pivotMotor::VOLTAGE_COMPENSATION);
    // Apply the configs to the motors
    m_pivotMotor.Configure(  m_pivotMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_pivotMotor.ClearFaults();

    m_pivotEncoder.SetDistancePerPulse(IntakeConstants::Encoder::DISTANCE_PER_PULSE);
    m_pivotEncoder.Reset();
}

void IntakeIOSpark::UpdateInputs(IntakeIOInputs& inputs) 
{
    inputs.isIntakeMotorConnected = (m_intakeMotor.GetBusVoltage() !=0.0) && !m_intakeMotor.GetFaults().can;

    inputs.intakeMotorAppliedVoltage = m_intakeMotor.GetAppliedOutput() * IntakeConstants::intakeMotor::VOLTAGE_COMPENSATION;
    inputs.intakeMotorBusVoltage = m_intakeMotor.GetBusVoltage();
    inputs.intakeMotorCurrent = m_intakeMotor.GetOutputCurrent();
    inputs.intakeMotorTemperature = m_intakeMotor.GetMotorTemperature();
    
    inputs.isPivotMotorConnected = (m_pivotMotor.GetBusVoltage() !=0.0) && !m_pivotMotor.GetFaults().can;

    inputs.pivotMotorAppliedVoltage = m_pivotMotor.GetAppliedOutput() * IntakeConstants::pivotMotor::VOLTAGE_COMPENSATION;
    inputs.pivotMotorBusVoltage = m_pivotMotor.GetBusVoltage();
    inputs.pivotMotorCurrent = m_pivotMotor.GetOutputCurrent();
    inputs.pivotMotorTemperature = m_pivotMotor.GetMotorTemperature();

    inputs.pivotPos = IntakeConstants::Specifications::STARTING_POS+m_pivotEncoder.GetDistance();

    frc::SmartDashboard::PutNumber("intake/IntakeMotorVelocity", m_intakeMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("intake/PivotPos", inputs.pivotPos);   
}

void IntakeIOSpark::SetVoltage(double intakeVoltage, double pivotVoltage)
{
    DEBUG_ASSERT((intakeVoltage <= IntakeConstants::intakeMotor::VOLTAGE_COMPENSATION) 
        && (intakeVoltage >= -IntakeConstants::intakeMotor::VOLTAGE_COMPENSATION) 
        ,"Intake Voltage out of range");
    DEBUG_ASSERT((pivotVoltage <= IntakeConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        && (pivotVoltage >= -IntakeConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        ,"Intake Voltage out of range");

    m_intakeMotor.SetVoltage(units::volt_t(intakeVoltage));
    m_pivotMotor.SetVoltage(units::volt_t(pivotVoltage));
}

void IntakeIOSpark::SetDutyCycle(double intakeDutyCycle, double pivotDutyCycle)
{
    DEBUG_ASSERT((intakeDutyCycle <= 1.0) && (intakeDutyCycle >= -1.0) 
        ,"Intake Duty Cycle out of range");
    DEBUG_ASSERT((pivotDutyCycle <= 1.0) && (pivotDutyCycle >= -1.0) 
        ,"Intake Duty Cycle out of range");
    m_intakeMotor.Set(pivotDutyCycle);
    m_pivotMotor.Set(pivotDutyCycle);
}

void IntakeIOSpark::ResetEncoder()
{
    m_pivotEncoder.Reset();
}

