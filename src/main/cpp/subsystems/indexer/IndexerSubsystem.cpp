#include "subsystems/indexer/indexerSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

IndexerSubsystem::IndexerSubsystem(IndexerIO *pIO) : 
                                                    m_pIndexerIO(pIO)
{
    m_indexerFeedforwardController.SetGains(IndexerConstants::Gains::FEEDFORWARD_VELOCITY_VOLTAGE::KS,
                                          IndexerConstants::Gains::FEEDFORWARD_VELOCITY_VOLTAGE::KV,
                                          IndexerConstants::Gains::FEEDFORWARD_VELOCITY_VOLTAGE::KA);
    m_indexerFeedforwardController.SetOutputLimits(IndexerConstants::Voltage::MIN, IndexerConstants::Voltage::MAX);
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
        case ControlMode::DISABLED :
            m_output = IndexerConstants::Voltage::REST;
            m_clodeOutput = IndexerConstants::Voltage::REST;

            m_controlMode = mode;
            break; //end of ControlMode::DISABLED

        case ControlMode::VOLTAGE:
            m_output = IndexerConstants::Voltage::REST;
            m_manualControlInput = IndexerConstants::Voltage::REST;
            m_clodeOutput = IndexerConstants::Voltage::REST;

            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = m_wantedState;

            m_controlMode = mode;
            break; //end of ControlMode::VOLTAGE

        case ControlMode::FEEDFORWARD_VELOCITY_VOLTAGE:
            m_output = IndexerConstants::Voltage::REST;
            m_targetVelocity = IndexerConstants::Voltage::REST;
            m_clodeOutput = IndexerConstants::Voltage::REST;

            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = m_wantedState;

            m_controlMode = mode;
            break; //end of ControlMode::VELOCITY_DUTYCYCLE_PID

        case ControlMode::MANUAL_VOLTAGE:
            m_output = IndexerConstants::Voltage::REST;
            m_manualControlInput = IndexerConstants::Voltage::REST;
            m_clodeOutput = IndexerConstants::Voltage::REST;

            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = m_wantedState;

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
    m_clodeMotorDisconnected.Set(!inputs.isClodeMotorConnected);
    m_clodeMotorHot.Set(inputs.clodeTemperature > IndexerConstants::clodeMotor::HOT_THRESHOLD);
    m_clodeMotorOverheating.Set(inputs.clodeTemperature > IndexerConstants::clodeMotor::OVERHEATING_THRESHOLD);
    
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
            case ControlMode::VOLTAGE:
                switch(m_systemState)
                {
                    case SystemState::IDLE:
                    case SystemState::SLEEPING:
                    case SystemState::READY_TO_SHOOT:
                        m_output = IndexerConstants::Voltage::REST;
                        m_clodeOutput = IndexerConstants::Voltage::REST;
                        break;

                    case SystemState::FEEDING_SHOOTER:
                        m_output = IndexerConstants::Voltage::FEED;
                        m_clodeOutput = IndexerConstants::Voltage::CLODE_POWER;
                        break;

                    case SystemState::EVACUATING_SHOOTER:
                        m_targetVelocity = IndexerConstants::Speed::EVACUATE;
                        m_output = m_indexerFeedforwardController.Calculate(0.0,m_targetVelocity,0.0);
                        m_clodeOutput = IndexerConstants::Voltage::CLODE_POWER;
                        break;

                    case SystemState::PREPARING_SHOOT:
                        m_targetVelocity = IndexerConstants::Speed::PREPARE_SHOOT;
                        m_output = m_indexerFeedforwardController.Calculate(0.0,m_targetVelocity,0.0);
                        m_clodeOutput = IndexerConstants::Voltage::CLODE_POWER;
                        break;

                    default:
                        m_targetVelocity = IndexerConstants::Speed::REST;
                        m_output = IndexerConstants::Voltage::REST;
                        m_clodeOutput = IndexerConstants::Voltage::REST;
                        DEBUG_ASSERT(false, "Indexer : unknown system state");
                        break;
                }
                break;

            case ControlMode::DISABLED :
                m_output = IndexerConstants::Voltage::REST;
                m_clodeOutput = IndexerConstants::Voltage::REST;
                break; //end of ControlMode::DISABLED

            case ControlMode::FEEDFORWARD_VELOCITY_VOLTAGE:
                switch(m_systemState)
                {
                    case SystemState::IDLE:
                    case SystemState::SLEEPING:
                    case SystemState::READY_TO_SHOOT:
                        m_targetVelocity = IndexerConstants::Speed::REST;
                        m_output = IndexerConstants::Voltage::REST;
                        m_clodeOutput = IndexerConstants::Voltage::REST;
                        break;

                    case SystemState::FEEDING_SHOOTER:
                        m_targetVelocity = IndexerConstants::Speed::FEED;
                        m_output = m_indexerFeedforwardController.Calculate(0.0,m_targetVelocity,0.0);
                        m_clodeOutput = IndexerConstants::Voltage::CLODE_POWER;
                        break;

                    case SystemState::EVACUATING_SHOOTER:
                        m_targetVelocity = IndexerConstants::Speed::EVACUATE;
                        m_output = m_indexerFeedforwardController.Calculate(0.0,m_targetVelocity,0.0);
                        m_clodeOutput = IndexerConstants::Voltage::CLODE_POWER;
                        break;

                    case SystemState::PREPARING_SHOOT:
                        m_targetVelocity = IndexerConstants::Speed::PREPARE_SHOOT;
                        m_output = m_indexerFeedforwardController.Calculate(0.0,m_targetVelocity,0.0);
                        m_clodeOutput = IndexerConstants::Voltage::CLODE_POWER;
                        break;

                    default:
                        m_targetVelocity = IndexerConstants::Speed::REST;
                        m_output = IndexerConstants::Voltage::REST;
                        m_clodeOutput = IndexerConstants::Voltage::REST;
                        DEBUG_ASSERT(false, "Indexer : unknown system state");
                        break;
                }
                break;

            case ControlMode::MANUAL_VOLTAGE:
                m_output = m_tunableVoltageLogger.Get();
                m_clodeOutput = m_manualControlInput;
                break;
                
            default:
                DEBUG_ASSERT(false, "Indexer : impossible state");
                break; //end of default
        }
    }


     // ----------------- Limits -----------------
    

    // Apply output
    switch (m_controlMode)
    {
        case ControlMode::DISABLED:
        case ControlMode::MANUAL_VOLTAGE:
        case ControlMode::FEEDFORWARD_VELOCITY_VOLTAGE:
            m_pIndexerIO->SetVoltage(m_output, m_clodeOutput);
            break;

        default:
            DEBUG_ASSERT(false, "Indexer : unknown control mode used");
            break;
    }

    //LOG
    frc::SmartDashboard::PutNumber("indexer/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("indexer/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("indexer/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutBoolean("indexer/isInit", m_isInitialized);
    frc::SmartDashboard::PutBoolean("indexer/IRBreaker", inputs.isThereABall);
    frc::SmartDashboard::PutNumber("indexer/Motor Voltage", m_output);
    frc::SmartDashboard::PutNumber("indexer/Target Velocity", m_targetVelocity);
    frc::SmartDashboard::PutNumber("indexer/Clode Voltage ", m_clodeOutput);
    frc::SmartDashboard::PutNumber("indexer/Motor speed", inputs.indexerMotorSpeed);
    frc::SmartDashboard::PutNumber("You have shot ", inputs.nbrOfBallShot);
}

void IndexerSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
        case WantedState::STAND_BY :
            if (m_systemState != SystemState::SLEEPING && m_systemState != SystemState::READY_TO_SHOOT)
                m_systemState = SystemState::SLEEPING;
            break; //end of Others States

        case WantedState::EVACUATE_SHOOTER:
            if (m_systemState != SystemState::EVACUATING_SHOOTER)
            {
                m_systemState = SystemState::EVACUATING_SHOOTER;
            }
            break;

        case WantedState::FEED_SHOOTER:
            if (m_systemState != SystemState::FEEDING_SHOOTER)
                m_systemState = SystemState::FEEDING_SHOOTER;
            break;

        case WantedState::PREPARE_SHOOT:
            if(m_systemState != SystemState::READY_TO_SHOOT)
            {
                m_systemState = SystemState::PREPARING_SHOOT;
            }
            break;

        default:
            DEBUG_ASSERT(false, "indexer : impossible wanted state");
            break; //end of default
    }

    switch (m_systemState) // Change System State
    {
        case SystemState::IDLE:
            if (inputs.isThereABall)
            {
                m_systemState = SystemState::READY_TO_SHOOT;
            }
            else
            {
                m_systemState = SystemState::SLEEPING;
            }

        case SystemState::SLEEPING:
        case SystemState::FEEDING_SHOOTER:
        case SystemState::READY_TO_SHOOT:
            break;

        case SystemState::EVACUATING_SHOOTER:
            if (!inputs.isThereABall && inputs.wasThereABall)
            {
                m_systemState = SystemState::IDLE;
            }
            break;

        case SystemState::PREPARING_SHOOT:
            if (inputs.isThereABall)
            {
                m_systemState = SystemState::READY_TO_SHOOT;
                m_wantedState = WantedState::STAND_BY;
                m_currentWantedState = m_wantedState;
            }
            break;

        default:
            DEBUG_ASSERT(false, "indexer : impossible system state");
            break; //end of default
    }
}