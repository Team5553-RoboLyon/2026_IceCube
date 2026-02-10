#include "subsystems/intake/pivot/PivotIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"

PivotIOSpark::PivotIOSpark()
{
    // Set the motor configs
    m_pivotMotorConfig.SetIdleMode(PivotConstants::pivotMotor::IDLE_MODE)
                      .Inverted(PivotConstants::pivotMotor::INVERTED)
                      .SmartCurrentLimit(PivotConstants::pivotMotor::CURRENT_LIMIT)
                      .ClosedLoopRampRate(PivotConstants::pivotMotor::RAMP_RATE)
                      .VoltageCompensation(PivotConstants::pivotMotor::VOLTAGE_COMPENSATION)
                      .closedLoop.P(PivotConstants::Gains::PIVOT_POSITION_DUTYCYCLE_PID::KP)
                      .I(PivotConstants::Gains::PIVOT_POSITION_DUTYCYCLE_PID::KI)
                      .D(PivotConstants::Gains::PIVOT_POSITION_DUTYCYCLE_PID::KD);

    // Apply the configs to the motors
    m_pivotMotor.Configure(  m_pivotMotorConfig, 
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    m_pivotMotor.ClearFaults();

    m_rightEncoder.SetDistancePerPulse(PivotConstants::EncoderRight::DISTANCE_PER_PULSE);
    m_leftEncoder.SetDistancePerPulse(PivotConstants::EncoderLeft::DISTANCE_PER_PULSE);
}

void PivotIOSpark::UpdateInputs(PivotIOInputs& inputs) 
{    
    inputs.isPivotMotorConnected = (m_pivotMotor.GetBusVoltage() !=0.0) && !m_pivotMotor.GetFaults().can;

    inputs.pivotMotorAppliedVoltage = m_pivotMotor.GetAppliedOutput() * PivotConstants::pivotMotor::VOLTAGE_COMPENSATION;
    inputs.pivotMotorBusVoltage = m_pivotMotor.GetBusVoltage();
    inputs.pivotMotorCurrent = m_pivotMotor.GetOutputCurrent();
    inputs.pivotMotorTemperature = m_pivotMotor.GetMotorTemperature();

    inputs.pivotPos = PivotConstants::Specifications::STARTING_POS+(m_leftEncoder.GetDistance()+m_rightEncoder.GetDistance())/2;
    frc::SmartDashboard::PutNumber("intake/PivotPos", inputs.pivotPos);
    frc::SmartDashboard::PutNumber("intake/PivotTargetPos", m_pivotMotorController.GetSetpoint());
}

void PivotIOSpark::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= PivotConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        && (voltage >= -PivotConstants::pivotMotor::VOLTAGE_COMPENSATION) 
        ,"Pivot Voltage out of range");

    m_pivotMotor.SetVoltage(units::volt_t(voltage));
}

void PivotIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
        ,"Intake Duty Cycle out of range");
    m_pivotMotor.Set(dutyCycle);    
}

void PivotIOSpark::SetTargetPos(double targetPos)
{
    m_pivotMotorController.SetSetpoint(targetPos,rev::spark::SparkLowLevel::ControlType::kPosition);
}

void PivotIOSpark::ResetEncoder()
{
    m_rightEncoder.Reset();
    m_leftEncoder.Reset();
}

