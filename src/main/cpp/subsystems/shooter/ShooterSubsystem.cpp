#include "subsystems/shooter/shooterSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

ShooterSubsystem::ShooterSubsystem(FlywheelIO *pFlywheelIO, HoodIO *pHoodIO) : 
                                                    m_pFlywheelIO(pFlywheelIO), m_pHoodIO(pHoodIO)
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

void ShooterSubsystem::SetControlMode(const ControlMode flywheelMode, const ControlMode hoodMode)
{
    SetFlywheelControlMode(flywheelMode);
    SetHoodControlMode(hoodMode);
}

void ShooterSubsystem::SetFlywheelControlMode(const ControlMode mode)
{
    switch (mode)
    {
        case ControlMode::VELOCITY_VOLTAGE_PID:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;
            m_systemState = SystemState::REST;
            m_wantedState = WantedState::STAND_BY;

            m_flywheelControlMode = mode;
            break; //end of ControlMode::VELOCITY_VOLTAGE_PID

        case ControlMode::VOLTAGE:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;
            m_systemState = SystemState::REST;
            m_wantedState = WantedState::STAND_BY;

            m_flywheelControlMode = mode;
            break; //end of ControlMode::VOLTAGE
        
        case ControlMode::DISABLED:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;

            m_flywheelControlMode = mode;
            break; //end of ControlMode::DISABLED

        default:
            DEBUG_ASSERT(false," Shooter (Flywheel): SetControlMode impossible with an unrecognized mode.");
            break; //end of default
    }
}

void ShooterSubsystem::SetHoodControlMode(const ControlMode mode)
{
    switch(mode)
    {
        case ControlMode::POSITION_DUTYCYCLE_PID:
            m_hoodOutput = HoodConstants::Voltage::REST;
            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;

            m_hoodControlMode = mode;
            break;  //end of ControlMode::POSITION_DUTYCYCLE_PID

        case ControlMode::VOLTAGE:
            m_hoodOutput = HoodConstants::Voltage::REST;
            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;

            m_hoodControlMode = mode;
            break;  //end of ControlMode::VOLTAGE

        case ControlMode::DISABLED:
            m_hoodOutput = HoodConstants::Voltage::REST;

            m_hoodControlMode = mode;
            break; //end of ControlMode::DISABLED

        default:
            DEBUG_ASSERT(false," Shooter (Hood): SetControlMode impossible with an unrecognized mode.");
            break; //end of default
    }
}

ControlMode ShooterSubsystem::GetFlywheelControlMode()
{
    return m_flywheelControlMode;
}

ControlMode ShooterSubsystem::GetHoodControlMode()
{
    return m_hoodControlMode;
}

void ShooterSubsystem::ToggleFlywheelControlMode()
{
    switch (m_flywheelControlMode)
    {
        case FlywheelConstants::MainControlMode :
            SetFlywheelControlMode(FlywheelConstants::EmergencyControlMode);
            break; //end of FlywheelConstants::MainControlMode
        case FlywheelConstants::EmergencyControlMode : 
            SetFlywheelControlMode(FlywheelConstants::MainControlMode);
            break; //end of FlywheelConstants::EmergencyControlMode
        default:
            DEBUG_ASSERT(false," Shooter (Flywheel) : Toggle impossible with an unrecognized mode.");
            break; //end of default
    }
}

void ShooterSubsystem::ToggleHoodControlMode()
{
    switch (m_hoodControlMode)
    {
        case HoodConstants::MainControlMode :
            SetHoodControlMode(HoodConstants::EmergencyControlMode);
            break; //end of HoodConstants::MainControlMode
        case HoodConstants::EmergencyControlMode : 
            SetHoodControlMode(HoodConstants::MainControlMode);
            break; //end of HoodConstants::EmergencyControlMode
        default:
            DEBUG_ASSERT(false," Shooter (Hood): Toggle impossible with an unrecognized mode.");
            break; //end of default
    }
}

void ShooterSubsystem::SetManualControlInput(const double value)
{
    if(BYPASS_STATE_MACHINE(m_flywheelControlMode) || BYPASS_STATE_MACHINE(m_hoodControlMode))
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

    m_pFlywheelIO->UpdateInputs(flywheelInputs);
    m_pHoodIO->UpdateInputs(hoodInputs);
    m_logger.Log(hoodInputs,flywheelInputs);
    
    m_leftMotorDisconnected.Set(!flywheelInputs.isLeftMotorConnected);
    m_leftMotorHot.Set(flywheelInputs.leftMotorTemperature > FlywheelConstants::LeftMotor::HOT_THRESHOLD);
    m_leftMotorOverheating.Set(flywheelInputs.leftMotorTemperature > FlywheelConstants::LeftMotor::OVERHEATING_THRESHOLD);
    m_rightMotorDisconnected.Set(!flywheelInputs.isRightMotorConnected);
    m_rightMotorHot.Set(flywheelInputs.rightMotorTemperature > FlywheelConstants::RightMotor::HOT_THRESHOLD);
    m_rightMotorOverheating.Set(flywheelInputs.rightMotorTemperature > FlywheelConstants::RightMotor::OVERHEATING_THRESHOLD);
    m_hoodMotorDisconnected.Set(!hoodInputs.isHoodMotorConnected);
    m_hoodMotorHot.Set(hoodInputs.hoodMotorTemperature > HoodConstants::HoodMotor::HOT_THRESHOLD);
    m_hoodMotorOverheating.Set(hoodInputs.hoodMotorTemperature > HoodConstants::HoodMotor::OVERHEATING_THRESHOLD);

   
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


        //LOG
    frc::SmartDashboard::PutNumber("shooter/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("shooter/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("shooter/Flywheel/ControlMode", (int)m_flywheelControlMode);
    frc::SmartDashboard::PutNumber("shooter/Hood/ControlMode", (int)m_hoodControlMode);
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