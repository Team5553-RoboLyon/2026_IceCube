#include "subsystems/indexer/indexerSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

IndexerSubsystem::IndexerSubsystem(IndexerIO *pIO) : 
                                                    m_pIndexerIO(pIO)
{
}

void IndexerSubsystem::SetWantedState(const WantedState wantedState)
{
        m_wantedState = wantedState;
}

IndexerSubsystem::SystemState IndexerSubsystem::GetSystemState()
{
    return m_systemState;
}

void IndexerSubsystem::SetControlMode(const ControlMode mode)
{
    switch (mode)
    {
    
    case ControlMode::MANUAL_DUTY_CYCLE:
        m_output = IndexerConstants::Speed::REST;
        m_manualControlInput = IndexerConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_DUTY_CYCLE
    
    case ControlMode::DISABLED :
        m_output = IndexerConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::DISABLED

    case ControlMode::MANUAL_VOLTAGE:
        m_output = IndexerConstants::Speed::REST;
        m_manualControlInput = IndexerConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::MANUAL_VOLTAGE

    default:
        DEBUG_ASSERT(false," Indexer : SetControlMode impossible with an unrecognized mode.");
        break; //end of default
    }
}

ControlMode IndexerSubsystem::GetControlMode()
{
    return m_controlMode;
}

void IndexerSubsystem::ToggleControlMode()
{
    switch (m_controlMode)
    {
    case IndexerConstants::MainControlMode :
        SetControlMode(IndexerConstants::EmergencyControlMode);
        break; //end of IndexerConstants::MainControlMode
    case IndexerConstants::EmergencyControlMode : 
        SetControlMode(IndexerConstants::MainControlMode);
        break; //end of IndexerConstants::EmergencyControlMode
    default:
        DEBUG_ASSERT(false," Indexer : Toggle impossible with an unrecognized mode.");
        break; //end of default
    }
}

void IndexerSubsystem::SetManualControlInput(const double value)
{
    if(BYPASS_STATE_MACHINE(m_controlMode))
    {
        DEBUG_ASSERT((value <= 1.0) && (value >= -1.0) 
            , "Indexer Duty Cycle out of range");
        m_manualControlInput = value;
    }
    else 
    {
        DEBUG_ASSERT(false, "Indexer : Open Loop Output set while Closed Loop is used");
    }
}

// This method will be called once per scheduler run
void IndexerSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pIndexerIO->UpdateInputs(inputs);
    m_logger.Log(inputs);
    
        m_indexerMotorDisconnected.Set(!inputs.isindexerMotorConnected);
    m_indexerMotorHot.Set(inputs.indexerMotorTemperature > IndexerConstants::indexerMotor::HOT_THRESHOLD);
    m_indexerMotorOverheating.Set(inputs.indexerMotorTemperature > IndexerConstants::indexerMotor::OVERHEATING_THRESHOLD);
    
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

        case ControlMode::MANUAL_VOLTAGE:
            m_output = m_tunableVoltageLogger.Get();
            break; //end of ControlMode::MANUAL_VOLTAGE

        case ControlMode::DISABLED :
            m_output = IndexerConstants::Speed::REST;
            break; //end of ControlMode::DISABLED
        default:
            DEBUG_ASSERT(false, "Indexer : impossible state");
            break; //end of default
        }
    }


     // ----------------- Limits -----------------
    

    // Apply output
    m_pIndexerIO->SetDutyCycle(m_output);



        //LOG
    frc::SmartDashboard::PutNumber("indexer/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("indexer/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("indexer/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutBoolean("indexer/isInit", m_isInitialized);
}

void IndexerSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::STAND_BY :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "indexer : impossible state");
        break; //end of default
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        break; //end of SystemState::IDLE
    default:
        DEBUG_ASSERT(false, "indexer : impossible state");
        break; //end of default
    }
}