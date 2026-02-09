#include "subsystems/climber/climberSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

ClimberSubsystem::ClimberSubsystem(ClimberIO *pIO) : 
                                                    m_pClimberIO(pIO)
{

    if(m_controlMode == ControlMode::POSITION_VOLTAGE_PID)
    {
        m_ClimberPIDController.SetGains(ClimberConstants::Gains::POSITION_VOLTAGE_PID::KP, 
                                ClimberConstants::Gains::POSITION_VOLTAGE_PID::KI, 
                                ClimberConstants::Gains::POSITION_VOLTAGE_PID::KD);
    }
    else if(m_controlMode == ControlMode::MANUAL_POSITION)
    {
        m_ClimberPIDController.SetGains(ClimberConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                ClimberConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                ClimberConstants::Gains::MANUAL_SETPOINT_PID::KD);
    }

    m_ClimberPIDController.Reset(m_timestamp);
    m_ClimberPIDController.SetOutputLimits(-ClimberConstants::Motor::VOLTAGE_COMPENSATION, ClimberConstants::Motor::VOLTAGE_COMPENSATION);
    m_ClimberPIDController.SetInputLimits(true);
    m_ClimberPIDController.SetInputLimits( ClimberConstants::Settings::BOTTOM_LIMIT, 
                                            ClimberConstants::Settings::TOP_LIMIT);

}

void ClimberSubsystem::SetWantedState(const WantedState wantedState)
{
    if(wantedState == WantedState::INITIALIZATION)
    {
        if(!m_isInitialized)  // Skip initialization if the subsystem is already initialized
            m_wantedState = WantedState::INITIALIZATION; 
    }
    else // if(wantedState != WantedState::INITIALIZATION)
    {
        m_wantedState = wantedState;
    }
}

ClimberSubsystem::SystemState ClimberSubsystem::GetSystemState()
{
    return m_systemState;
}

void ClimberSubsystem::SetControlMode(const ControlMode mode)
{
    switch (mode)
    {
    case ControlMode::POSITION_VOLTAGE_PID:
        m_output = ClimberConstants::Speed::REST;
        m_wantedState = WantedState::STAND_BY;
        m_currentWantedState = m_wantedState;
        m_systemState = SystemState::IDLE;

        m_ClimberPIDController.Reset(m_timestamp);
        m_ClimberPIDController.SetGains(ClimberConstants::Gains::POSITION_VOLTAGE_PID::KP, 
                                ClimberConstants::Gains::POSITION_VOLTAGE_PID::KI, 
                                ClimberConstants::Gains::POSITION_VOLTAGE_PID::KD);
        m_controlMode = mode;
        break; //end of ControlMode::POSITION_VOLTAGE_PID
    
    case ControlMode::MANUAL_POSITION :
        m_output = ClimberConstants::Speed::REST;
        m_manualControlInput = inputs.climberHeight;

        m_ClimberPIDController.Reset(m_timestamp);
        m_ClimberPIDController.SetGains(ClimberConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                ClimberConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                ClimberConstants::Gains::MANUAL_SETPOINT_PID::KD);

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_POSITION
    
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
    
    m_climberMotorDisconnected.Set(!inputs.isMotorConnected);
    m_climberMotorHot.Set(inputs.motorTemperature > ClimberConstants::Motor::HOT_THRESHOLD);
    m_climberMotorOverheating.Set(inputs.motorTemperature > ClimberConstants::Motor::OVERHEATING_THRESHOLD);
    
    if(!m_isInitialized)
    {
        if(m_currentWantedState == WantedState::INITIALIZATION)
        {
            m_output = ClimberConstants::Speed::CALIBRATION;
        }
    }
    else 
    {
        if(ALLOWS_STATE_MACHINE(m_controlMode))
        {
            RunStateMachine();
        }

        switch (m_controlMode) //actualise motion
        {
        case ControlMode::MANUAL_VOLTAGE:
            m_output = m_tunableVoltage.Get();
            break; //end of ControlMode::MANUAL_VOLTAGE
        case ControlMode::POSITION_VOLTAGE_PID:
            switch (m_systemState)
            {
                case SystemState::ARMED:
                case SystemState::EXTENDING_TO_ARMED:
                    m_output = m_ClimberPIDController.Calculate(ClimberConstants::Setpoint::ARMED,inputs.climberHeight);
                    break; //end of SystemState::ARMED and SystemState::EXTENDING_TO_ARMED
                case SystemState::CLIMBED_LOCKED:
                case SystemState::CLIMBING:
                    m_output = m_ClimberPIDController.Calculate(ClimberConstants::Setpoint::CLIMBED_LOCKED,inputs.climberHeight);
                    break; //end of SystemState::CLIMBED_LOCKED and SystemState::CLIMBING
                case SystemState::STOWED_HOME:
                case SystemState::RETRACTING_TO_HOME:
                    m_output = m_ClimberPIDController.Calculate(ClimberConstants::Setpoint::HOME,inputs.climberHeight);
                    break; //end of SystemState::STOWED_HOME and SystemState::RETRACTING_TO_HOME
                
                case SystemState::IDLE:
                    m_output = ClimberConstants::Speed::REST;
                    break; //end of SystemState::IDLE
                default:
                    DEBUG_ASSERT(false, "Climber : impossible state");
                    break;
            }
            break; //end of ControlMode::POSITION_VOLTAGE_PID
        case ControlMode::MANUAL_POSITION :
            m_manualControlInput = m_ClimberPIDController.GetSetpoint() + m_manualControlInput * ClimberConstants::Settings::MANUAL_SETPOINT_CHANGE_LIMIT;

            m_output = m_ClimberPIDController.CalculateWithRealTime(m_manualControlInput,
                                                                        inputs.climberHeight,
                                                                        m_timestamp);
            break; //end of ControlMode::MANUAL_POSITION
        case ControlMode::DISABLED :
            m_output = ClimberConstants::Speed::REST;
            break; //end of ControlMode::DISABLED
        default:
            DEBUG_ASSERT(false, "Climber : impossible state");
            break; //end of default
        }
    }


     // ----------------- Limits -----------------
    if (inputs.climberHeight >= ClimberConstants::Settings::TOP_LIMIT && m_output > 0.0)
        m_output = 0.0;
    else if (inputs.climberHeight <= ClimberConstants::Settings::BOTTOM_LIMIT && m_output < 0.0)
        m_output = 0.0;
    
    if(inputs.bottomLimitSwitchValue)
    {
        m_output = NMAX(0.0, m_output); // prevent the elevator to go through the bottom
        if(!m_isEncoderAlreadyReset)
        {
            m_pClimberIO->ResetPosition();
            m_isEncoderAlreadyReset = true; // prevent the encoder to reset many times
            if(!m_isInitialized)
            {
                m_isInitialized = true;
                m_wantedState = WantedState::STAND_BY;
                m_currentWantedState = WantedState::STAND_BY;
            }
        }
    }
    else if (inputs.climberHeight > ClimberConstants::Settings::TOP_LIMIT)
    {
        m_output = NMIN(0.0, m_output); // prevent the straffer to go through the right side
    }
    else 
    {
        m_isEncoderAlreadyReset = false; // allow the encoder to be reset at the bottom if it goes up
    }

    // Apply output
    m_pClimberIO->SetVoltage(m_output);

        //LOG
    frc::SmartDashboard::PutNumber("climber/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("climber/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("climber/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutNumber("climber/Height", inputs.climberHeight);
    frc::SmartDashboard::PutBoolean("climber/isInit", m_isInitialized);
}

void ClimberSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::STOWED :
        if (m_systemState == SystemState::ARMED || 
            m_systemState == SystemState::EXTENDING_TO_ARMED)
        {
            m_systemState = SystemState::RETRACTING_TO_HOME;
        }
        break; //end of WantedState::STOWED
    case WantedState::ARMED_TO_CLIMB :
        if (m_systemState == SystemState::STOWED_HOME || 
            m_systemState == SystemState::RETRACTING_TO_HOME ||
            m_systemState == SystemState::CLIMBED_LOCKED ||
            m_systemState == SystemState::CLIMBING)
        {
            m_systemState = SystemState::EXTENDING_TO_ARMED;
        }
        break; //end of WantedState::ARMED_TO_CLIMB
    case WantedState::CLIMBED:
        if( m_systemState == SystemState::ARMED)
        {
            m_systemState = SystemState::CLIMBING;
        }
        break; //end of WantedState::CLIMBED
    case WantedState::RELEASED_CLIMB:
        if( m_systemState == SystemState::CLIMBED_LOCKED)
        {
            m_systemState = SystemState::EXTENDING_TO_ARMED;
        }
        break; //end of WantedState::RELEASED_CLIMB
    case WantedState::STAND_BY :
    case WantedState::INITIALIZATION :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "climber : impossible state");
        break; //end of default
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        m_systemState = SystemState::RETRACTING_TO_HOME;
        break; //end of SystemState::IDLE

    case SystemState::CLIMBING :
        if(NABS(inputs.climberHeight - ClimberConstants::Setpoint::CLIMBED_LOCKED) < ClimberConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::CLIMBED_LOCKED;
        }
        break; //end of SystemState::CLIMBED
    case SystemState::RETRACTING_TO_HOME :
        if(NABS(inputs.climberHeight - ClimberConstants::Setpoint::HOME) < ClimberConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::STOWED_HOME;
        }
        break; //end of SystemState::RETRACTING_TO_HOME
    case SystemState::EXTENDING_TO_ARMED :
        if(NABS(inputs.climberHeight - ClimberConstants::Setpoint::ARMED) < ClimberConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::ARMED;
        }
        break; //end of SystemState::EXTENDING_TO_ARMED
    case SystemState::STOWED_HOME:
    case SystemState::ARMED:
    case SystemState::CLIMBED_LOCKED:
        break; //end of Steady states
    default:
        DEBUG_ASSERT(false, "climber : impossible state");
        break; //end of default
    }
}