#include "subsystems/turret/turretSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

TurretSubsystem::TurretSubsystem(TurretIO *pIO) : 
                                                    m_pTurretIO(pIO)
{
    if(m_controlMode == ControlMode::MANUAL_POSITION)
    {
        m_TurretPIDController.SetGains(TurretConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                TurretConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                TurretConstants::Gains::MANUAL_SETPOINT_PID::KD);
        m_TurretPIDController.SetTolerance(TurretConstants::Gains::MANUAL_SETPOINT_PID::TOLERANCE);
    }

    m_TurretPIDController.Reset(m_timestamp);
    m_TurretPIDController.SetOutputLimits(TurretConstants::Speed::MIN, TurretConstants::Speed::MAX);
    m_TurretPIDController.SetInputLimits(true);
    m_TurretPIDController.SetInputLimits( TurretConstants::Settings::BOTTOM_LIMIT, 
                                            TurretConstants::Settings::TOP_LIMIT);
}

void TurretSubsystem::SetWantedState(const WantedState wantedState)
{
        m_wantedState = wantedState;
}

TurretSubsystem::SystemState TurretSubsystem::GetSystemState()
{
    return m_systemState;
}

void TurretSubsystem::SetControlMode(const ControlMode mode)
{
    switch (mode)
    {
    
    case ControlMode::MANUAL_DUTY_CYCLE:
        m_output = TurretConstants::Speed::REST;
        m_manualControlInput = TurretConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_DUTY_CYCLE
    case ControlMode::MANUAL_POSITION:
        m_output = TurretConstants::Speed::REST;
        m_manualControlInput = inputs.orientation;

        m_TurretPIDController.Reset(m_timestamp);
        m_TurretPIDController.SetGains(TurretConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                TurretConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                TurretConstants::Gains::MANUAL_SETPOINT_PID::KD);
        m_TurretPIDController.SetTolerance(TurretConstants::Gains::MANUAL_SETPOINT_PID::TOLERANCE);
        m_TurretPIDController.SetFeedforward(TurretConstants::Gains::MANUAL_SETPOINT_PID::KG);

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_POSITION
    case ControlMode::DISABLED :
        m_output = TurretConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::DISABLED

    default:
        DEBUG_ASSERT(false," Turret : SetControlMode impossible with an unrecognized mode.");
        break; //end of default
    }
}

ControlMode TurretSubsystem::GetControlMode()
{
    return m_controlMode;
}

void TurretSubsystem::ToggleControlMode()
{
    switch (m_controlMode)
    {
    case TurretConstants::MainControlMode :
        SetControlMode(TurretConstants::EmergencyControlMode);
        break; //end of TurretConstants::MainControlMode
    case TurretConstants::EmergencyControlMode : 
        SetControlMode(TurretConstants::MainControlMode);
        break; //end of TurretConstants::EmergencyControlMode
    default:
        DEBUG_ASSERT(false," Turret : Toggle impossible with an unrecognized mode.");
        break; //end of default
    }
}

void TurretSubsystem::SetManualControlInput(const double value)
{
    if(BYPASS_STATE_MACHINE(m_controlMode))
    {
        DEBUG_ASSERT((value <= 1.0) && (value >= -1.0) 
            , "Turret Duty Cycle out of range");
        m_manualControlInput = value;
    }
    else 
    {
        DEBUG_ASSERT(false, "Turret : Open Loop Output set while Closed Loop is used");
    }
}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pTurretIO->UpdateInputs(inputs);
    m_logger.Log(inputs);
    
    m_motorDisconnected.Set(!inputs.ismotorConnected);
    m_motorHot.Set(inputs.motorTemperature > TurretConstants::motor::HOT_THRESHOLD);
    m_motorOverheating.Set(inputs.motorTemperature > TurretConstants::motor::OVERHEATING_THRESHOLD);
    
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

        case ControlMode::MANUAL_POSITION :
            m_manualControlInput = m_TurretPIDController.GetSetpoint() 
                                + m_manualControlInput * TurretConstants::Settings::MANUAL_SETPOINT_CHANGE_LIMIT;

            m_output = m_TurretPIDController.CalculateWithRealTime(m_manualControlInput,
                                                                        inputs.orientation,
                                                                        m_timestamp);
            break; //end of ControlMode::MANUAL_POSITION
        case ControlMode::DISABLED :
            m_output = TurretConstants::Speed::REST;
            break; //end of ControlMode::DISABLED
        default:
            DEBUG_ASSERT(false, "Turret : impossible state");
            break; //end of default
        }
    }


    // ----------------- Limits -----------------
    if(inputs.orientation < TurretConstants::Settings::BOTTOM_LIMIT)
    {
        m_output = NMAX(0.0, m_output); // prevent the turret to go through the bottom side
    }
    else if(inputs.orientation > TurretConstants::Settings::TOP_LIMIT)
    {
        m_output = NMIN(0.0, m_output); // prevent the turret to go through the top side
    }

    // Apply output
    m_pTurretIO->SetDutyCycle(m_output);



        //LOG
    frc::SmartDashboard::PutNumber("turret/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("turret/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("turret/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutBoolean("turret/isInit", m_isInitialized);
}

void TurretSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::STAND_BY :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "turret : impossible state");
        break; //end of default
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        break; //end of SystemState::IDLE
    default:
        DEBUG_ASSERT(false, "turret : impossible state");
        break; //end of default
    }
}