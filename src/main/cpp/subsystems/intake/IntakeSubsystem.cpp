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
    
    case ControlMode::MANUAL_DUTY_CYCLE:
        m_output = IntakeConstants::Speed::REST;
        m_manualControlInput = IntakeConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_DUTY_CYCLE
    
    case ControlMode::DISABLED :
        m_output = IntakeConstants::Speed::REST;

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

void IntakeSubsystem::SetManualControlInput(const std::function<double()> Axis)
{
    m_fxAxis = Axis;
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pIntakeIO->UpdateInputs(inputs);
    m_logger.Log(inputs);

    m_manualControlInput = m_fxAxis();
    
        m_leftMotorDisconnected.Set(!inputs.isleftMotorConnected);
    m_leftMotorHot.Set(inputs.leftMotorTemperature > IntakeConstants::leftMotor::HOT_THRESHOLD);
    m_leftMotorOverheating.Set(inputs.leftMotorTemperature > IntakeConstants::leftMotor::OVERHEATING_THRESHOLD);
    m_rightMotorDisconnected.Set(!inputs.isrightMotorConnected);
    m_rightMotorHot.Set(inputs.rightMotorTemperature > IntakeConstants::rightMotor::HOT_THRESHOLD);
    m_rightMotorOverheating.Set(inputs.rightMotorTemperature > IntakeConstants::rightMotor::OVERHEATING_THRESHOLD);
    
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
        case ControlMode::MANUAL_DUTY_CYCLE :
            m_output = m_manualControlInput;
            break; //end of ControlMode::MANUAL_DUTY_CYCLE

        case ControlMode::DISABLED :
            m_output = IntakeConstants::Speed::REST;
            break; //end of ControlMode::DISABLED
        default:
            DEBUG_ASSERT(false, "Intake : impossible state");
            break; //end of default
        }
    }


     // ----------------- Limits -----------------
    

    // Apply output
    m_pIntakeIO->SetDutyCycle(m_output);



        //LOG
    frc::SmartDashboard::PutNumber("intake/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("intake/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("intake/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutBoolean("intake/isInit", m_isInitialized);
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