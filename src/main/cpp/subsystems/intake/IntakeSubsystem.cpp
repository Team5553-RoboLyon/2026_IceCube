#include "subsystems/intake/intakeSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

IntakeSubsystem::IntakeSubsystem(IntakeIO *pIO) : 
                                                    m_pIntakeIO(pIO)
{
}

void IntakeSubsystem::SetWantedState(const WantedState wantedState)
{
        m_wantedState = wantedState;
}

IntakeSubsystem::SystemState IntakeSubsystem::GetSystemState()
{
    return m_systemState;
}

void IntakeSubsystem::SetControlMode(const ControlMode mode)
{
    switch (mode)
    {
    case ControlMode::MANUAL_VOLTAGE:
        m_intakeOutput = 0.0;
        m_pivotOutput = 0.0;
        m_michelOutput = 0.0;
        m_manualControlInput = 0.0;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_VOLTAGE
        
    case ControlMode::MANUAL_VELOCITY:
        m_intakeOutput = IntakeConstants::Speed::REST;
        m_pivotOutput = IntakeConstants::Speed::REST;
        m_michelOutput = IntakeConstants::Speed::REST;
        m_manualControlInput = IntakeConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_VELOCITY
    
    case ControlMode::DISABLED :
        m_intakeOutput = IntakeConstants::Speed::REST;
        m_pivotOutput = IntakeConstants::Speed::REST;
        m_michelOutput = IntakeConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::DISABLED

    default:
        DEBUG_ASSERT(false," Intake : SetControlMode impossible with an unrecognized mode.");
        break; //end of default
    }
}

ControlMode IntakeSubsystem::GetControlMode()
{
    return m_controlMode;
}

void IntakeSubsystem::ToggleControlMode()
{
    switch (m_controlMode)
    {
    case IntakeConstants::MainControlMode :
        SetControlMode(IntakeConstants::EmergencyControlMode);
        break; //end of IntakeConstants::MainControlMode
    case IntakeConstants::EmergencyControlMode : 
        SetControlMode(IntakeConstants::MainControlMode);
        break; //end of IntakeConstants::EmergencyControlMode
    default:
        DEBUG_ASSERT(false," Intake : Toggle impossible with an unrecognized mode.");
        break; //end of default
    }
}

// void IntakeSubsystem::SetManualControlInput(const double value)
// {
//     if(BYPASS_STATE_MACHINE(m_controlMode))
//     {
//         DEBUG_ASSERT((value <= 1.0) && (value >= -1.0) 
//             , "Intake Duty Cycle out of range");
//         m_manualControlInput = value;
//     }
//     else 
//     {
//         DEBUG_ASSERT(false, "Intake : Open Loop Output set while Closed Loop is used");
//     }
// }

// void IntakeSubsystem::SetManualControlInput(const std::function<double()> Axis)
// {
//     m_fxAxis = Axis;
// }

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pIntakeIO->UpdateInputs(inputs);
    m_logger.Log(inputs);

    // m_manualControlInput = m_fxAxis();
    
    m_intakeMotorDisconnected.Set(!inputs.isIntakeMotorConnected);
    m_intakeMotorHot.Set(inputs.intakeMotorTemperature > IntakeConstants::intakeMotor::HOT_THRESHOLD);
    m_intakeMotorOverheating.Set(inputs.intakeMotorTemperature > IntakeConstants::intakeMotor::OVERHEATING_THRESHOLD);
    m_pivotMotorDisconnected.Set(!inputs.isPivotMotorConnected);
    m_pivotMotorHot.Set(inputs.pivotMotorTemperature > IntakeConstants::pivotMotor::HOT_THRESHOLD);
    m_pivotMotorOverheating.Set(inputs.pivotMotorTemperature > IntakeConstants::pivotMotor::OVERHEATING_THRESHOLD);
    m_michelMotorDisconnected.Set(!inputs.isMichelMotorConnected);
    m_michelMotorHot.Set(inputs.michelTemperature > IntakeConstants::michelMotor::HOT_THRESHOLD);
    m_michelMotorOverheating.Set(inputs.michelTemperature > IntakeConstants::michelMotor::OVERHEATING_THRESHOLD);
    
    if(!m_isInitialized)
    {
    }
    else 
    {
        if(ALLOWS_STATE_MACHINE(m_controlMode))
        {
            RunStateMachine();
        }

        switch (m_controlMode) //actualise motion
        {
        case ControlMode::MANUAL_VELOCITY:
            m_intakeOutput = m_tunableIntakeVoltageLogger.Get()/IntakeConstants::Specifications::intakeMotor_FREE_SPEED * IntakeConstants::intakeMotor::VOLTAGE_COMPENSATION;
            m_pivotOutput = m_tunablePivotVoltageLogger.Get()/IntakeConstants::Specifications::pivotMotor_FREE_SPEED * IntakeConstants::pivotMotor::VOLTAGE_COMPENSATION;
            break; //end of ControlMode::MANUAL_VELOCITY

        case ControlMode::DISABLED :
            m_intakeOutput = IntakeConstants::Speed::REST;
            m_pivotOutput = IntakeConstants::Speed::REST;
            break; //end of ControlMode::DISABLED
        case ControlMode::MANUAL_VOLTAGE:
            m_intakeOutput = m_tunableIntakeVoltageLogger.Get();
            m_pivotOutput = m_tunablePivotVoltageLogger.Get();
            m_michelOutput = m_tunableMichelVoltageLogger.Get();
            break;
        default:
            DEBUG_ASSERT(false, "Intake : impossible state");
            break; //end of default
        }
    }


     // ----------------- Limits -----------------
    if (m_pivotOutput > 0.0 && inputs.pivotPos <= IntakeConstants::Specifications::PIVOT_MIN_EXTENSION)
    {
        m_pivotOutput = 0.0;
    }
    else if (m_pivotOutput < 0.0 && inputs.pivotPos >= IntakeConstants::Specifications::PIVOT_MAX_EXTENSION)
    {
        m_pivotOutput = 0.0;
    }

    // Apply output
    m_pIntakeIO->SetVoltage(m_intakeOutput, m_pivotOutput, m_michelOutput);

        //LOG
    frc::SmartDashboard::PutNumber("intake/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("intake/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("intake/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutBoolean("intake/isInit", m_isInitialized);
    frc::SmartDashboard::PutNumber("intake/intakeOutput", m_intakeOutput);
    frc::SmartDashboard::PutNumber("intake/pivotVoltage", inputs.pivotMotorAppliedVoltage);
}

void IntakeSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::STAND_BY :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "intake : impossible state");
        break; //end of default
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        break; //end of SystemState::IDLE
    default:
        DEBUG_ASSERT(false, "intake : impossible state");
        break; //end of default
    }
}

void IntakeSubsystem::ResetEncoder()
{
    m_pIntakeIO->ResetEncoder();
}