#include "subsystems/turret/turretSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"
#include "LyonLib/Utils/MacroUtilsRBL.h"


TurretSubsystem::TurretSubsystem(TurretIO *pIO, ShootParameters *pShootParams) : 
                                                    m_pTurretIO(pIO),
                                                    m_pShootParams(pShootParams)
{
    m_TurretPIDController.SetGains(TurretConstants::Gains::POSITION_DUTYCYCLE_PID::KP, 
                                TurretConstants::Gains::POSITION_DUTYCYCLE_PID::KI, 
                                TurretConstants::Gains::POSITION_DUTYCYCLE_PID::KD);

    m_TurretPIDController.Reset(m_timestamp);
    m_TurretPIDController.SetOutputLimits(TurretConstants::DutyCycle::MIN, TurretConstants::DutyCycle::MAX);
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
        case ControlMode::POSITION_DUTYCYCLE_PID:
            m_output = TurretConstants::DutyCycle::REST;
            m_targetPos = inputs.orientation;

            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = m_wantedState;
            m_systemState = SystemState::IDLE;

            m_controlMode = mode;
            break; //end of ControlMode::MANUAL_DUTY_CYCLE

        case ControlMode::MANUAL_POSITION:
            m_output = TurretConstants::DutyCycle::REST;
            m_manualControlInput = TurretConstants::DutyCycle::REST;
            m_targetPos = inputs.orientation;

            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = m_wantedState;
            m_systemState = SystemState::IDLE;

            m_controlMode = mode;
            break; //end of ControlMode::MANUAL_POSITION

        case ControlMode::DISABLED :
            m_output = TurretConstants::DutyCycle::REST;

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
    // m_turretCamera.UpdateResults();
    
    m_motorDisconnected.Set(!inputs.isMotorConnected);
    m_motorHot.Set(inputs.motorTemperature > TurretConstants::Motor::HOT_THRESHOLD);
    m_motorOverheating.Set(inputs.motorTemperature > TurretConstants::Motor::OVERHEATING_THRESHOLD);
    
    if(!m_isInitialized)
    {
        // m_output = TurretConstants::DutyCycle::INIT;
        // if(inputs.hallEffectSensorValue > m_highestHallEffectSensorValue)
        // {
        //     m_highestHallEffectSensorValue = inputs.hallEffectSensorValue;
        //     m_pTurretIO->ResetOrientation();
        // }
        // else if (inputs.hallEffectSensorValue < TurretConstants::HallEffectSensor::MIN_VALUE_WHEN_MAGNET &&
        //          m_highestHallEffectSensorValue >= TurretConstants::HallEffectSensor::MIN_VALUE_WHEN_MAGNET)
        // {
        //     m_isInitialized = true;
        //     m_output = TurretConstants::DutyCycle::REST;
        // }
    }
    else 
    {
        if(ALLOWS_STATE_MACHINE(m_controlMode))
        {
            RunStateMachine();
        }

        switch (m_controlMode) //actualise motion
        {
            case ControlMode::POSITION_DUTYCYCLE_PID :

                switch (m_systemState)
                {
                    case SystemState::IDLE:
                    case SystemState::INACTIVE:
                    case SystemState::READY_TO_EJECT:
                        m_output = 0.0;
                        break;

                    case SystemState::ALIGNED_WITH_HUB:
                    case SystemState::POINTING_AT_ALLIANCE_ZONE:
                    case SystemState::ALIGNING_WITH_HUB:
                    case SystemState::ALIGNING_WITH_ALLIANCE_ZONE:
                    case SystemState::SPINNING_TO_EJECT:
                        m_output = m_TurretPIDController.CalculateWithRealTime(m_targetPos,inputs.orientation,m_timestamp);
                        break;

                    default:
                        DEBUG_ASSERT(false, "Turret : unknown system state used");
                        m_output = 0.0;
                        break;
                }
                break;

            case ControlMode::MANUAL_POSITION :
                m_manualControlInput = m_TurretPIDController.GetSetpoint() 
                                    + m_manualControlInput * TurretConstants::Settings::MANUAL_SETPOINT_CHANGE_LIMIT;

                m_output = m_TurretPIDController.CalculateWithRealTime(m_manualControlInput,
                                                                            inputs.orientation,
                                                                            m_timestamp);

                    // m_TurretPIDController.SetGains(m_tunableKP.Get(),0.0,0.0);
                break; //end of ControlMode::MANUAL_POSITION

            case ControlMode::DISABLED :
                m_output = TurretConstants::DutyCycle::REST;
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

    frc::SmartDashboard::PutNumber("turret/Target", m_TurretPIDController.GetSetpoint());

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
            if (m_systemState != SystemState::INACTIVE)
                m_systemState = SystemState::INACTIVE;
            break; //end of Others States

        case WantedState::FOLLOW_HUB:
            if (m_systemState != SystemState::ALIGNED_WITH_HUB)
                m_systemState = SystemState::ALIGNING_WITH_HUB;
            break;

        case WantedState::POINT_AT_ALLIANCE_ZONE:
            if (m_systemState != SystemState::POINTING_AT_ALLIANCE_ZONE)
                m_systemState = SystemState::ALIGNING_WITH_ALLIANCE_ZONE;
            break;

        // case WantedState::PREPARE_EJECT:
        //     if (m_systemState != SystemState::READY_TO_EJECT)
        //         m_systemState = SystemState::SPINNING_TO_EJECT;
        //     break;

        default:
            DEBUG_ASSERT(false, "turret : impossible state");
            break; //end of default
    }

    switch (m_systemState) // Change System State
    {
        case SystemState::IDLE:
        case SystemState::INACTIVE:
        case SystemState::READY_TO_EJECT:
            break; //end of SystemState::IDLE 
                   //   and SystemState::INACTIVE

        case SystemState::ALIGNING_WITH_HUB:
        case SystemState::ALIGNED_WITH_HUB:
            // if(inputs.orientation < 0.0)
            // {
            //     m_targetPos = m_pShootParams->lookAheadTargetTurretPos - 2.0*NF64_PI;
            // }
            // else
            // {
            //     m_targetPos = m_pShootParams->lookAheadTargetTurretPos;
            // }

            // if (inputs.orientation >= 0.0 && m_targetPos - inputs.orientation > NF64_PI)
            // {
            //     m_targetPos -= 2*NF64_PI;
            // }
            // else if (inputs.orientation <= 0.0 && m_targetPos - inputs.orientation < -NF64_PI)
            // {
            //     m_targetPos += 2*NF64_PI;
            // }

            m_targetPos = m_pShootParams->lookAheadTargetTurretPos;

            if (IS_IN_RANGE(inputs.orientation, m_targetPos, TurretConstants::Setpoints::TOLERANCE))
                m_systemState = SystemState::ALIGNED_WITH_HUB;
            else
                m_systemState = SystemState::ALIGNING_WITH_HUB;
            break;

        case SystemState::ALIGNING_WITH_ALLIANCE_ZONE:
        case SystemState::POINTING_AT_ALLIANCE_ZONE:

            // if (inputs.orientation < 0.0)
            // {
            //     m_targetPos = m_pShootParams->lookAheadTargetTurretPos - 2.0*NF64_PI;
            // }
            // else
            // {
            //     m_targetPos = m_pShootParams->lookAheadTargetTurretPos;
            // }

            // if (inputs.orientation >= 0.0 && m_targetPos - inputs.orientation > NF64_PI)
            // {
            //     m_targetPos -= 2*NF64_PI;
            // }
            // else if (inputs.orientation <= 0.0 && m_targetPos - inputs.orientation < -NF64_PI)
            // {
            //     m_targetPos += 2*NF64_PI;
            // }

            m_targetPos = m_pShootParams->lookAheadTargetTurretPos;

            if(IS_IN_RANGE(inputs.orientation, m_targetPos, TurretConstants::Setpoints::TOLERANCE))
                m_systemState = SystemState::POINTING_AT_ALLIANCE_ZONE;
            else
            {
                m_systemState = SystemState::ALIGNING_WITH_ALLIANCE_ZONE;
            }
            break;

        case SystemState::SPINNING_TO_EJECT:
            m_targetPos = TurretConstants::Setpoints::EJECT;
            if(IS_IN_RANGE(inputs.orientation, m_targetPos, TurretConstants::Setpoints::TOLERANCE))
                m_systemState = SystemState::READY_TO_EJECT;
            break;
        
        default:
            DEBUG_ASSERT(false, "turret : impossible state");
            break; //end of default
    }
}