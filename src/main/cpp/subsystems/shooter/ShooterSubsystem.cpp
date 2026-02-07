#include "subsystems/shooter/shooterSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

ShooterSubsystem::ShooterSubsystem(ShooterIO *pIO) : 
                                                    m_pShooterIO(pIO)
{
}

void ShooterSubsystem::SetWantedState(const WantedState wantedState)
{
        m_wantedState = wantedState;
}

ShooterSubsystem::SystemState ShooterSubsystem::GetSystemState()
{
    return m_systemState;
}

void ShooterSubsystem::SetControlMode(const ControlMode mode)
{
    switch (mode)
    {
    
    case ControlMode::VOLTAGE:
        m_upVoltage = ShooterConstants::Speed::REST;
        m_bottomVoltage = ShooterConstants::Speed::REST;
        m_systemState = SystemState::REST;
        m_wantedState = WantedState::STAND_BY;

        m_controlMode = mode;
        break; //end of ControlMode::VOLTAGE
    
    case ControlMode::DISABLED :
        m_upVoltage = ShooterConstants::Speed::REST;
        m_bottomVoltage = ShooterConstants::Speed::REST;

        m_controlMode = mode;
        break; //end of ControlMode::DISABLED

    default:
        DEBUG_ASSERT(false," Shooter : SetControlMode impossible with an unrecognized mode.");
        break; //end of default
    }
}

ControlMode ShooterSubsystem::GetControlMode()
{
    return m_controlMode;
}

void ShooterSubsystem::ToggleControlMode()
{
    switch (m_controlMode)
    {
    case ShooterConstants::MainControlMode :
        SetControlMode(ShooterConstants::EmergencyControlMode);
        break; //end of ShooterConstants::MainControlMode
    case ShooterConstants::EmergencyControlMode : 
        SetControlMode(ShooterConstants::MainControlMode);
        break; //end of ShooterConstants::EmergencyControlMode
    default:
        DEBUG_ASSERT(false," Shooter : Toggle impossible with an unrecognized mode.");
        break; //end of default
    }
}

void ShooterSubsystem::SetManualControlInput(const double value)
{
    if(BYPASS_STATE_MACHINE(m_controlMode))
    {
        DEBUG_ASSERT((value <= 1.0) && (value >= -1.0) 
            , "Shooter Duty Cycle out of range");
        m_manualControlInput = value;
    }
    else 
    {
        DEBUG_ASSERT(false, "Shooter : Open Loop Output set while Closed Loop is used");
    }
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pShooterIO->UpdateInputs(inputs);
    m_logger.Log(inputs);
    
    m_LeftMotorDisconnected.Set(!inputs.isLeftMotorConnected);
    m_LeftMotorHot.Set(inputs.LeftMotorTemperature > ShooterConstants::LeftMotor::HOT_THRESHOLD);
    m_LeftMotorOverheating.Set(inputs.LeftMotorTemperature > ShooterConstants::LeftMotor::OVERHEATING_THRESHOLD);
    m_RightMotorDisconnected.Set(!inputs.isRightMotorConnected);
    m_RightMotorHot.Set(inputs.RightMotorTemperature > ShooterConstants::RightMotor::HOT_THRESHOLD);
    m_RightMotorOverheating.Set(inputs.RightMotorTemperature > ShooterConstants::RightMotor::OVERHEATING_THRESHOLD);
    m_BottomMotorDisconnected.Set(!inputs.isRightMotorConnected);
    m_BottomMotorHot.Set(inputs.RightMotorTemperature > ShooterConstants::RightMotor::HOT_THRESHOLD);
    m_BottomMotorOverheating.Set(inputs.RightMotorTemperature > ShooterConstants::RightMotor::OVERHEATING_THRESHOLD);

   
    // DEBUG_ASSERT(m_targetVelocity >= -ShooterConstants::Specifications::LeftMotor_FREE_SPEED && 
    //                 m_targetVelocity <= ShooterConstants::Specifications::LeftMotor_FREE_SPEED
    //     , "Shooter Target Velocity out of range");

     // ----------------- State Machine & Motion Control -----------------
    
    // if(!m_isInitialized)
    // {
    // }
    // else 
    // {
    //     if(ALLOWS_STATE_MACHINE(m_controlMode))
    //     {
    //         RunStateMachine();
    //     }

    m_upVoltage = m_tunableUpVoltageLogger.Get();
    m_bottomVoltage = m_tunableBottomVoltageLogger.Get();

        // switch (m_controlMode) //actualise motion
        // {
        // case ControlMode::VOLTAGE :
        //     //  m_upVoltage = m_tunableUpVoltageLogger.Get();
        //     //  m_bottomVoltage = m_tunableBottomVoltageLogger.Get();
        //     // switch (m_systemState)
        //     // {
        //     // case SystemState::SHOOTING :
        //     //     m_bottomOutput = 
        //     //     break; //end of SystemState::SHOOTING
        //     // default:
        //     //     m_output = ShooterConstants::Speed::REST;
        //     //     break; //end of other States
        //     // }
        //     // break; //end of ControlMode::VOLTAGE

        // case ControlMode::DISABLED :
        //     m_upVoltage = 0.0;
        //     m_bottomVoltage = 0.0;
        //     // m_output = ShooterConstants::Speed::REST;
        //     // break; //end of ControlMode::DISABLED
        // default:
        //     // DEBUG_ASSERT(false, "Shooter : impossible state");
        //     break; //end of default
        // }
    // }


     // ----------------- Limits -----------------
    

    // Apply output
    if (m_controlMode == ControlMode::VOLTAGE)
        m_pShooterIO->SetVoltage(m_upVoltage,m_bottomVoltage);
    else
        m_pShooterIO->SetVoltage(0.0,0.0);


        //LOG
    frc::SmartDashboard::PutNumber("shooter/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("shooter/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("shooter/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutBoolean("shooter/isInit", m_isInitialized);
}

void ShooterSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::STAND_BY :
        break; //end of Others States
    case WantedState::SHOOT :
        m_systemState = SystemState::SHOOTING;
        break; //end of WantedState::SHOOT
    case WantedState::STOP :
        m_systemState = SystemState::REST;
        break; //end of WantedState::STOP
    default:
        DEBUG_ASSERT(false, "shooter : impossible state");
        break; //end of default
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        break; //end of SystemState::IDLE
    case SystemState::SHOOTING:
        break; //end of SystemState::SHOOTING
    case SystemState::REST:
        break; //end of SystemState::REST
    default:
        DEBUG_ASSERT(false, "shooter : impossible state");
        break; //end of default
    }
}