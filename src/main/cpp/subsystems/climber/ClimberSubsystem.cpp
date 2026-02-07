#include "subsystems/climber/climberSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

ClimberSubsystem::ClimberSubsystem(ClimberIO *pIO) : 
                                                    m_pClimberIO(pIO)
{
}

void ClimberSubsystem::SetWantedState(const WantedState wantedState)
{
        m_wantedState = wantedState;
}

ClimberSubsystem::SystemState ClimberSubsystem::GetSystemState()
{
    return m_systemState;
}

void ClimberSubsystem::SetControlMode(const ControlMode mode)
{
    switch (mode)
    {
    
    case ControlMode::MANUAL_DUTY_CYCLE:
        m_output = ClimberConstants::Speed::REST;
        m_manualControlInput = ClimberConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_DUTY_CYCLE
    
    case ControlMode::MANUAL_VOLTAGE:
        m_output = ClimberConstants::Speed::REST;
        m_manualControlInput = ClimberConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_VOLTAGE
    
    case ControlMode::DISABLED :
        m_output = ClimberConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::DISABLED

    default:
        DEBUG_ASSERT(false," Climber : SetControlMode impossible with an unrecognized mode.");
        break; //end of default
    }
}

ControlMode ClimberSubsystem::GetControlMode()
{
    return m_controlMode;
}

void ClimberSubsystem::ToggleControlMode()
{
    switch (m_controlMode)
    {
    case ClimberConstants::MainControlMode :
        SetControlMode(ClimberConstants::EmergencyControlMode);
        break; //end of ClimberConstants::MainControlMode
    case ClimberConstants::EmergencyControlMode : 
        SetControlMode(ClimberConstants::MainControlMode);
        break; //end of ClimberConstants::EmergencyControlMode
    default:
        DEBUG_ASSERT(false," Climber : Toggle impossible with an unrecognized mode.");
        break; //end of default
    }
}

void ClimberSubsystem::SetManualControlInput(const double value)
{
    if(BYPASS_STATE_MACHINE(m_controlMode))
    {
        DEBUG_ASSERT((value <= 1.0) && (value >= -1.0) 
            , "Climber Duty Cycle out of range");
        m_manualControlInput = value;
    }
    else 
    {
        DEBUG_ASSERT(false, "Climber : Open Loop Output set while Closed Loop is used");
    }
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pClimberIO->UpdateInputs(inputs);
    m_logger.Log(inputs);
    
        m_climberMotorDisconnected.Set(!inputs.isclimberMotorConnected);
    m_climberMotorHot.Set(inputs.climberMotorTemperature > ClimberConstants::climberMotor::HOT_THRESHOLD);
    m_climberMotorOverheating.Set(inputs.climberMotorTemperature > ClimberConstants::climberMotor::OVERHEATING_THRESHOLD);
    
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
            m_output = ClimberConstants::Speed::REST;
            break; //end of ControlMode::DISABLED

        case ControlMode::MANUAL_VOLTAGE:
            m_output = m_tunableVoltage.Get();
            break;

        default:
            DEBUG_ASSERT(false, "Climber : impossible state");
            break; //end of default
        }
    }


     // ----------------- Limits -----------------
    if (inputs.climberPos >= ClimberConstants::Specifications::MAX_POSITION && m_output > 0.0)
        m_output = 0.0;
    else if (inputs.climberPos <= ClimberConstants::Specifications::MIN_POSITION && m_output < 0.0)
        m_output = 0.0;

    // Apply output
    m_pClimberIO->SetDutyCycle(m_output);

        //LOG
    frc::SmartDashboard::PutNumber("climber/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("climber/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("climber/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutNumber("climber/AppliedCurrent", inputs.climberMotorCurrent);
    frc::SmartDashboard::PutNumber("climber/Pos", inputs.climberPos);
    frc::SmartDashboard::PutBoolean("climber/isInit", m_isInitialized);
}

void ClimberSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::STAND_BY :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "climber : impossible state");
        break; //end of default
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        break; //end of SystemState::IDLE
    default:
        DEBUG_ASSERT(false, "climber : impossible state");
        break; //end of default
    }
}

void ClimberSubsystem::ResetEncoder()
{
    m_pClimberIO->ResetEncoder();
}