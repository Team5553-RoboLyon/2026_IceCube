#include "subsystems/intake/intakeSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

IntakeSubsystem::IntakeSubsystem(RollerIO *pRollerIO, PivotIO *pPivotIO) : 
                                                    m_pRollerIO(pRollerIO), m_pPivotIO(pPivotIO)
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

void IntakeSubsystem::SetControlMode(const ControlMode pivotMode, const ControlMode rollerMode)
{
    SetPivotControlMode(pivotMode);
    SetRollerControlMode(rollerMode);
}

void IntakeSubsystem::SetPivotControlMode(ControlMode mode)
{
    switch (mode)
    {
        case ControlMode::POSITION_DUTYCYCLE_PID:
            m_pivotOutput = 0.0;
            m_pivotTargetPos = pivotInputs.pivotPos;

            m_pivotControlMode = mode;
            break; //end of ControlMode::POSITION_DUTYCYCLE_PID

        case ControlMode::MANUAL_VOLTAGE:
            m_pivotOutput = 0.0;
            m_pivotManualControlInput = 0.0;

            m_pivotControlMode = mode;
            break; //end of ControlMode::MANUAL_VOLTAGE

        case ControlMode::DISABLED :
            m_pivotOutput = PivotConstants::Speed::REST;

            m_pivotControlMode = mode;
            break; //end of ControlMode::DISABLED

        default:
            DEBUG_ASSERT(false,"Pivot : SetControlMode impossible with an unrecognized mode.");
            break; //end of default
    }
}

void IntakeSubsystem::SetRollerControlMode(ControlMode mode)
{
    switch(mode)
    {
        case ControlMode::MANUAL_VOLTAGE:
            m_rollerOutput = 0.0;
            m_rollerManualControlInput = 0.0;

            m_rollerControlMode = mode;
            break; //end of ControlMode::MANUAL_VOLTAGE

        case ControlMode::VOLTAGE:
            m_rollerOutput = 0.0;
            m_rollerManualControlInput = 0.0;

            m_rollerControlMode = mode;
            break; //end of ControlMode::VOLTAGE
        
        case ControlMode::DISABLED :
            m_rollerOutput = RollerConstants::Speed::REST;

            m_rollerControlMode = mode;
            break; //end of ControlMode::DISABLED

        default:
            DEBUG_ASSERT(false,"Pivot : SetControlMode impossible with an unrecognized mode.");
            break; //end of default
    }
}

ControlMode IntakeSubsystem::GetPivotControlMode()
{
    return m_pivotControlMode;
}

ControlMode IntakeSubsystem::GetRollerControlMode()
{
    return m_rollerControlMode;
}

void IntakeSubsystem::TogglePivotControlMode()
{
    switch (m_pivotControlMode)
    {
        case PivotConstants::MainControlMode :
            SetPivotControlMode(PivotConstants::EmergencyControlMode);
            break; //end of IntakeConstants::MainControlMode

        case PivotConstants::EmergencyControlMode : 
            SetPivotControlMode(PivotConstants::MainControlMode);
            break; //end of IntakeConstants::EmergencyControlMode

        default:
            DEBUG_ASSERT(false," Pivot : Toggle impossible with an unrecognized mode.");
            break; //end of default
    }
}

void IntakeSubsystem::ToggleRollerControlMode()
{
    switch (m_rollerControlMode)
    {
        case RollerConstants::MainControlMode :
            SetRollerControlMode(RollerConstants::EmergencyControlMode);
            break; //end of IntakeConstants::MainControlMode

        case RollerConstants::EmergencyControlMode : 
            SetRollerControlMode(RollerConstants::MainControlMode);
            break; //end of IntakeConstants::EmergencyControlMode

        default:
            DEBUG_ASSERT(false," Pivot : Toggle impossible with an unrecognized mode.");
            break; //end of default
    }
}

bool IntakeSubsystem::IsOut()
{
    return m_systemState == SystemState::CHILLING_OUT ||
           m_systemState == SystemState::REFUELING || 
           m_systemState == SystemState::EJECTING;
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
    // m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pRollerIO->UpdateInputs(rollerInputs);
    m_pPivotIO->UpdateInputs(pivotInputs);
    m_logger.Log(rollerInputs, pivotInputs);

    // m_manualControlInput = m_fxAxis();
    
    m_rollerMotorDisconnected.Set(!rollerInputs.isRollerMotorConnected);
    m_rollerMotorHot.Set(rollerInputs.rollerMotorTemperature > RollerConstants::rollerMotor::HOT_THRESHOLD);
    m_rollerMotorOverheating.Set(rollerInputs.rollerMotorTemperature > RollerConstants::rollerMotor::OVERHEATING_THRESHOLD);
    m_pivotMotorDisconnected.Set(!pivotInputs.isPivotMotorConnected);
    m_pivotMotorHot.Set(pivotInputs.pivotMotorTemperature > PivotConstants::pivotMotor::HOT_THRESHOLD);
    m_pivotMotorOverheating.Set(pivotInputs.pivotMotorTemperature > PivotConstants::pivotMotor::OVERHEATING_THRESHOLD);
    
    if(!m_isInitialized)
    {
    }
    else 
    {
        if(ALLOWS_STATE_MACHINE(m_pivotControlMode) || ALLOWS_STATE_MACHINE(m_rollerControlMode))
        {
            RunStateMachine();
        }

        switch (m_pivotControlMode) //actualise pivot motion
        {
            case ControlMode::DISABLED :
                m_pivotOutput = PivotConstants::Speed::REST;
                break; //end of ControlMode::DISABLED

            case ControlMode::MANUAL_VOLTAGE:
                m_pivotOutput = m_tunablePivotVoltageLogger.Get();
                break; //end of ControlMode::MANUAL_VOLTAGE

            case ControlMode::POSITION_DUTYCYCLE_PID:
                switch(m_systemState)
                {
                    case SystemState::CHILLING_OUT:
                    case SystemState::REFUELING:
                    case SystemState::EJECTING:
                    case SystemState::EXTENDING:
                        m_pivotTargetPos = PivotConstants::Specifications::PIVOT_EXTENDED_POS;
                        break;

                    case SystemState::IDLE:
                    case SystemState::FEELING_LIKE_AN_INDEXER:
                    case SystemState::STAYING_AT_HOME:
                    case SystemState::COMING_BACK_HOME:
                        m_pivotTargetPos = PivotConstants::Specifications::PIVOT_HOME_POS;
                        break;

                    default:
                        DEBUG_ASSERT(false, "Intake (Pivot) : unknown system state used");
                        break;
                }
                break;

            default:
                DEBUG_ASSERT(false, "Pivot : impossible state");
                break; //end of default
        }

        switch (m_rollerControlMode) //actualise roller motion
        {
            case ControlMode::DISABLED :
                m_rollerOutput = RollerConstants::Speed::REST;
                break; //end of ControlMode::DISABLED

            case ControlMode::MANUAL_VOLTAGE:
                m_rollerOutput = m_tunableRollerVoltageLogger.Get();
                break;

            case ControlMode::VOLTAGE:
                switch(m_systemState)
                {
                    case SystemState::IDLE:
                    case SystemState::STAYING_AT_HOME:
                    case SystemState::CHILLING_OUT:
                    case SystemState::EXTENDING:
                    case SystemState::COMING_BACK_HOME:
                        m_rollerOutput = RollerConstants::Voltage::REST;
                        break;

                    case SystemState::REFUELING:
                        m_rollerOutput = RollerConstants::Voltage::REFUEL;
                        break;

                    case SystemState::EJECTING:
                        m_rollerOutput = RollerConstants::Voltage::EJECT;
                        break;

                    case SystemState::FEELING_LIKE_AN_INDEXER:
                        m_rollerOutput = RollerConstants::Voltage::IM_AN_INDEXER;
                        break;

                    default:
                        DEBUG_ASSERT(false, "Intake (Roller) : unknown system state used");
                        break;
                }
                break;

            default:
                DEBUG_ASSERT(false, "Roller : impossible state");
                break; //end of default
        }
    }


     // ----------------- Limits -----------------
    if (m_pivotOutput > 0.0 && pivotInputs.pivotPos <= PivotConstants::Specifications::PIVOT_EXTENDED_POS)
    {
        m_pivotOutput = 0.0;
    }
    else if (m_pivotOutput < 0.0 && pivotInputs.pivotPos >= PivotConstants::Specifications::PIVOT_HOME_POS)
    {
        m_pivotOutput = 0.0;
    }

    // Apply output
    switch(m_pivotControlMode)
    {
        case ControlMode::DISABLED:
        case ControlMode::MANUAL_VOLTAGE:
            m_pPivotIO->SetVoltage(m_pivotOutput);
            break;

        case ControlMode::POSITION_DUTYCYCLE_PID:
            m_pPivotIO->SetTargetPos(m_pivotTargetPos);
            break;

        default:
            DEBUG_ASSERT(false,"Pivot : unknown conrtol mode");
            break;
    }
    switch(m_rollerControlMode)
    {
        case ControlMode::DISABLED:
        case ControlMode::MANUAL_VOLTAGE:
        case ControlMode::VOLTAGE:
            m_pRollerIO->SetVoltage(m_rollerOutput);
            break;

        default:
            DEBUG_ASSERT(false,"Roller : unknown conrtol mode");
            break;
    }

        //LOG
    frc::SmartDashboard::PutNumber("intake/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("intake/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("intake/PivotControlMode", (int)m_pivotControlMode);
    frc::SmartDashboard::PutNumber("intake/RollerControlMode", (int)m_rollerControlMode);
    frc::SmartDashboard::PutBoolean("intake/isInit", m_isInitialized);
    frc::SmartDashboard::PutNumber("intake/intakeOutput", m_rollerOutput);
    frc::SmartDashboard::PutNumber("intake/pivotVoltage", pivotInputs.pivotMotorAppliedVoltage);
}

void IntakeSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
        case WantedState::STAND_BY :
            if (m_systemState != SystemState::CHILLING_OUT || m_systemState != SystemState::STAYING_AT_HOME)
            {
                if (IsOut())
                {
                    m_systemState = SystemState::CHILLING_OUT;
                }
                else
                    m_systemState = SystemState::STAYING_AT_HOME;
            }
            break; //end of Others States 
            //TODO
        case WantedState::REFUEL:
            if (m_systemState != SystemState::REFUELING)
            {
                if(IsOut())
                {
                    m_systemState = SystemState::REFUELING;
                }
                else
                    m_systemState = SystemState::EXTENDING;
            }
            break;

        case WantedState::EJECT:
            if (m_systemState != SystemState::EJECTING)
            {
                if(IsOut())
                {
                    m_systemState = SystemState::EJECTING;
                }
                else
                    m_systemState = SystemState::EXTENDING;
            }
            break;

        case WantedState::BECOME_AN_INDEXER:
            if (m_systemState != SystemState::FEELING_LIKE_AN_INDEXER)
            {
                if(IsOut())
                {
                    m_systemState = SystemState::COMING_BACK_HOME;
                }
                else
                    m_systemState = SystemState::FEELING_LIKE_AN_INDEXER;
            }
            break;

        default:
            DEBUG_ASSERT(false, "intake : impossible state");
            break; //end of default
    }

    switch (m_systemState) // Change System State
    {
        case SystemState::IDLE:
        case SystemState::CHILLING_OUT:
        case SystemState::STAYING_AT_HOME:
        case SystemState::REFUELING:
        case SystemState::EJECTING:
        case SystemState::FEELING_LIKE_AN_INDEXER:
            break; //end of SystemState::IDLE
                   //    and SystemState::CHILLING_OUT
                   //    and SystemState::STAYING_AT_HOME
                   //    and SystemState::REFUELING
                   //    and SystemState::EJECTING
                   //    and SystemState::FEELING_LIKE_AN_INDEXER

        case SystemState::EXTENDING:
            if (pivotInputs.pivotPos <= PivotConstants::Specifications::PIVOT_EXTENDED_POS + PivotConstants::Specifications::POS_TOLERANCE
                && pivotInputs.pivotPos >= PivotConstants::Specifications::PIVOT_EXTENDED_POS - PivotConstants::Specifications::POS_TOLERANCE)
                {
                    m_systemState = SystemState::CHILLING_OUT;
                    m_wantedState = WantedState::STAND_BY;
                }
                break;

        case SystemState::COMING_BACK_HOME:
            if (pivotInputs.pivotPos <= PivotConstants::Specifications::PIVOT_HOME_POS + PivotConstants::Specifications::POS_TOLERANCE
                && pivotInputs.pivotPos >= PivotConstants::Specifications::PIVOT_HOME_POS - PivotConstants::Specifications::POS_TOLERANCE)
                {
                    m_systemState = SystemState::STAYING_AT_HOME;
                    m_wantedState = WantedState::STAND_BY;
                }
                break;

        default:
            DEBUG_ASSERT(false, "intake : impossible state");
            break; //end of default
    }
}

// void IntakeSubsystem::PivotResetEncoder()
// {
//     m_pPivotIO->ResetEncoder();
// }