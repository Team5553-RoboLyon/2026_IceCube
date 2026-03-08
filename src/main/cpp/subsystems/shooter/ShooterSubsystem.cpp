#include "subsystems/shooter/shooterSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "LyonLib/logging/DebugUtils.h"
#include "LyonLib/utils/TimerRBL.h"

ShooterSubsystem::ShooterSubsystem(FlywheelIO *pFlywheelIO, HoodIO *pHoodIO, ShootParameters* pShootParams) : 
                                                    m_pFlywheelIO(pFlywheelIO), m_pHoodIO(pHoodIO), m_pShootParameters(pShootParams) 
{
    m_flywheelPIDController.SetGains(FlywheelConstants::Gains::VELOCITY_VOLTAGE_PID::KP,
                                     FlywheelConstants::Gains::VELOCITY_VOLTAGE_PID::KI,
                                     FlywheelConstants::Gains::VELOCITY_VOLTAGE_PID::KD);

    m_flywheelPIDController.SetOutputLimits(double(FlywheelConstants::Voltage::MIN), double(FlywheelConstants::Voltage::MAX));
    m_flywheelPIDController.SetInputLimits(double(FlywheelConstants::Speed::MIN), double(FlywheelConstants::Speed::MAX));

    m_hoodPIDController.SetGains(HoodConstants::Gains::POSITION_VOLTAGE_PID::KP,
                                 HoodConstants::Gains::POSITION_VOLTAGE_PID::KI,
                                 HoodConstants::Gains::POSITION_VOLTAGE_PID::KD);

    m_hoodPIDController.SetOutputLimits(double(HoodConstants::Voltage::MIN), double(HoodConstants::Voltage::MAX));
    m_hoodPIDController.SetInputLimits(true);
    m_hoodPIDController.SetInputLimits(HoodConstants::Position::MIN, HoodConstants::Position::MAX);

    m_flywheelFeedforward.SetGains(FlywheelConstants::Gains::FLYWHEEL_FEEDFORWARD::KS,FlywheelConstants::Gains::FLYWHEEL_FEEDFORWARD::KV,0.0);
    m_flywheelFeedforward.SetOutputLimits(double(FlywheelConstants::Voltage::MIN), double(FlywheelConstants::Voltage::MAX));

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
        case ControlMode::VELOCITY_MODEL_CONTROLLED:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;
            m_systemState = SystemState::RESTING;
            m_wantedState = WantedState::STAND_BY;

            m_flywheelControlMode = mode;
            break; //end of ControlMode::VELOCITY_MODEL_CONTROLLED

        case ControlMode::VOLTAGE:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;
            m_systemState = SystemState::RESTING;
            m_wantedState = WantedState::STAND_BY;

            m_flywheelControlMode = mode;
            break; //end of ControlMode::VOLTAGE
        
        case ControlMode::MANUAL_VOLTAGE:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;

             m_flywheelControlMode = mode;
            break;
        
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
        case ControlMode::POSITION_VOLTAGE_PID:
            m_hoodOutput = HoodConstants::Voltage::REST;
            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;

            m_hoodPIDController.Reset(m_timestamp);
            m_hoodPIDController.SetGains(HoodConstants::Gains::POSITION_VOLTAGE_PID::KP, 
                                    HoodConstants::Gains::POSITION_VOLTAGE_PID::KI, 
                                    HoodConstants::Gains::POSITION_VOLTAGE_PID::KD);

            m_hoodControlMode = mode;
            break;  //end of ControlMode::POSITION_VOLTAGE_PID

        case ControlMode::MANUAL_POSITION:
            m_hoodOutput = HoodConstants::Voltage::REST;
            m_systemState = SystemState::IDLE;
            m_wantedState = WantedState::STAND_BY;

            m_manualControlInput = hoodInputs.hoodAngle;

            m_hoodPIDController.Reset(m_timestamp);
            m_hoodPIDController.SetGains(HoodConstants::Gains::MANUAL_POSITION::KP, 
                                    HoodConstants::Gains::MANUAL_POSITION::KI, 
                                    HoodConstants::Gains::MANUAL_POSITION::KD);

            m_hoodControlMode = mode;
            break;  //end of ControlMode::MANUAL_POSITION


        case ControlMode::DISABLED:
            m_hoodOutput = HoodConstants::Voltage::REST;

            m_hoodControlMode = mode;
            break; //end of ControlMode::DISABLED
        
        // case ControlMode::MANUAL_POSITION:

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
    switch(m_hoodControlMode)
    {
        case ControlMode::MANUAL_POSITION:
            m_manualControlInput = m_hoodPIDController.GetSetpoint() + value * HoodConstants::Settings::MANUAL_SETPOINT_CHANGE_LIMIT;
            break; //end of ControlMode::MANUAL_POSITION
        default:
            DEBUG_ASSERT(false, "Climber : SetManualControlInput impossible with an unrecognized mode.");
            break; //end of default
    }
}

bool ShooterSubsystem::IsHoodRetract()
{
    return IS_IN_RANGE(hoodInputs.hoodAngle, 0.0, HoodConstants::Position::TOLERANCE);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    m_pFlywheelIO->UpdateInputs(flywheelInputs);
    m_pHoodIO->UpdateInputs(hoodInputs);
    
    m_leftMotorDisconnected.Set(!flywheelInputs.isLeftMotorConnected);
    m_leftMotorHot.Set(flywheelInputs.leftMotorTemperature > FlywheelConstants::LeftMotor::HOT_THRESHOLD);
    m_leftMotorOverheating.Set(flywheelInputs.leftMotorTemperature > FlywheelConstants::LeftMotor::OVERHEATING_THRESHOLD);
    m_rightMotorDisconnected.Set(!flywheelInputs.isRightMotorConnected);
    m_rightMotorHot.Set(flywheelInputs.rightMotorTemperature > FlywheelConstants::RightMotor::HOT_THRESHOLD);
    m_rightMotorOverheating.Set(flywheelInputs.rightMotorTemperature > FlywheelConstants::RightMotor::OVERHEATING_THRESHOLD);

    m_hoodMotorDisconnected.Set(!hoodInputs.isMotorConnected);
    m_hoodMotorHot.Set(hoodInputs.motorTemperature > HoodConstants::HoodMotor::HOT_THRESHOLD);
    m_hoodMotorOverheating.Set(hoodInputs.motorTemperature > HoodConstants::HoodMotor::OVERHEATING_THRESHOLD);

     //----------------- State Machine & Motion Control -----------------
        
    if(ALLOWS_STATE_MACHINE(m_flywheelControlMode) || ALLOWS_STATE_MACHINE(m_hoodControlMode))
    {
        RunStateMachine();
    }

    switch (m_flywheelControlMode) //actualise flywheel motion
    {
        case ControlMode::VOLTAGE:
            m_flywheelOutput = units::volt_t{m_flywheelFeedforward.Calculate(0.0, m_flywheelTargetSpeed, 0.0)};
            break;
        case ControlMode::VELOCITY_MODEL_CONTROLLED:
            switch(m_systemState)
            {
                case SystemState::AT_SHOOT_SPEED:
                case SystemState::SHOOTING_BACKWARD:
                case SystemState::RAMPING_TO_SHOOT:
                case SystemState::RAMPING_BACKWARD:
                case SystemState::THATS_ALL_MINE:
                case SystemState::SOON_MINE:
                    {
                        double pid = m_flywheelPIDController.CalculateWithRealTime(m_flywheelTargetSpeed, 
                                                                                    flywheelInputs.shooterVelocity, 
                                                                                    m_timestamp);
                        double ff = m_flywheelFeedforward.Calculate(0.0, m_flywheelTargetSpeed, 0.0);
                        m_flywheelOutput = units::volt_t{
                            NCLAMP(
                                double(FlywheelConstants::Voltage::MIN),
                                pid + ff,
                                double(FlywheelConstants::Voltage::MAX))};
                    }
                    break;

                case SystemState::IDLE:
                case SystemState::RESTING:
                case SystemState::SLOWING_DOWN:
                case SystemState::RETRACTING_HOOD:
                    m_flywheelOutput = units::volt_t{m_flywheelFeedforward.Calculate(0.0, m_flywheelTargetSpeed, 0.0)};
                    break;

                default:
                    DEBUG_ASSERT(false, "Shooter (Flywheel) : unknown system state used");
                    m_flywheelOutput = FlywheelConstants::Voltage::REST;
                    break;
            }
            break;

        case ControlMode::DISABLED:
            m_flywheelOutput = FlywheelConstants::Voltage::REST;
            break;

        case ControlMode::MANUAL_VOLTAGE:
            {
                #if ROBOT_MODEL == PROTOTYPE
                    m_flywheelFeedforward.SetGains(FlywheelConstants::Gains::FLYWHEEL_FEEDFORWARD::KS, m_tunableFlywheelKVLogger.Get(), 0.0);
                    m_flywheelPIDController.SetGains(m_tunableFlywheelKPLogger.Get(), 0.0, m_tunableFlywheelKDLogger.Get());

                    double rpm = m_tunableFlywheelVoltageLogger.Get();
                    double ff = m_flywheelFeedforward.Calculate(0.0, rpm, 0.0);
                    double pid = m_flywheelPIDController.CalculateWithRealTime(
                                    rpm,
                                    flywheelInputs.shooterVelocity,
                                    m_timestamp);

                    m_flywheelOutput = units::volt_t{
                        NCLAMP(
                            double(FlywheelConstants::Voltage::MIN),
                            pid + ff,
                            double(FlywheelConstants::Voltage::MAX))

                    // m_flywheelOutput = units::volt_t{m_tunableFlywheelVoltageLogger.Get()};
                    };  
                #endif
                break;       
            }
        default: 
            DEBUG_ASSERT(false, "Shooter (Flywheel) : unknown control mode used");
            m_flywheelOutput = FlywheelConstants::Voltage::REST;
            break;
    }

    switch (m_hoodControlMode)
    {
        case ControlMode::POSITION_VOLTAGE_PID:
            if (m_wantedState == WantedState::PREPARE_SHOOT || m_wantedState == WantedState::PREPARE_TO_KEEP_ALL)
            {
                m_hoodTargetPos = 0.0;
            }
            m_hoodOutput = units::volt_t{m_hoodPIDController.CalculateWithRealTime(m_hoodTargetPos, hoodInputs.hoodAngle, m_timestamp)};
            break;

        case ControlMode::DISABLED:
            m_hoodOutput = HoodConstants::Voltage::REST;
            break;
        
        case ControlMode::MANUAL_POSITION:
            m_hoodOutput = units::volt_t{m_hoodPIDController.CalculateWithRealTime(m_manualControlInput, hoodInputs.hoodAngle, m_timestamp)};
            break;

        default:
            m_hoodOutput = HoodConstants::Voltage::REST;
            DEBUG_ASSERT(false, "Shooter (Hood) : unknown control mode used");
            break;
    }


    //  // ----------------- Limits -----------------
    
     if (hoodInputs.hoodAngle <= HoodConstants::Position::MIN && double(m_hoodOutput) < 0.0)
     {
        m_hoodOutput = HoodConstants::Voltage::REST;
     }
     else if (hoodInputs.hoodAngle   >= HoodConstants::Position::MAX && double(m_hoodOutput) > 0.0)
     {
        m_hoodOutput = HoodConstants::Voltage::REST;
     }
    // Apply output
    m_pFlywheelIO->SetVoltage(m_flywheelOutput);
    m_pHoodIO->SetVoltage(m_hoodOutput); 
    //LOG
    frc::SmartDashboard::PutNumber("shooter/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("shooter/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("shooter/flywheel/ControlMode", (int)m_flywheelControlMode);
    frc::SmartDashboard::PutNumber("shooter/hood/ControlMode", (int)m_hoodControlMode);
    frc::SmartDashboard::PutBoolean("shooter/isInit", m_isInitialized);
    frc::SmartDashboard::PutNumber("shooter/flywheel/velocityTarget", m_flywheelTargetSpeed);
    frc::SmartDashboard::PutNumber("shooter/hood/positionTarget", m_hoodPIDController.GetSetpoint());
}

void ShooterSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
        case WantedState::STAND_BY :
            m_systemState = SystemState::RESTING;
            break;

        case WantedState::SHOOT_TO_HUB:
            if (m_systemState != SystemState::AT_SHOOT_SPEED)
                m_systemState = SystemState::RAMPING_TO_SHOOT;
            else 
                m_wantedState = WantedState::STAND_BY; // ignoring the command
            break;

        case WantedState::STOP:
            if (m_systemState != SystemState::RESTING)
                m_systemState = SystemState::SLOWING_DOWN;
            else 
                m_wantedState = WantedState::STAND_BY; // ignoring the command
            break;

        case WantedState::REVERSE:
            if (m_systemState != SystemState::SHOOTING_BACKWARD)
                m_systemState = SystemState::RAMPING_BACKWARD;
            else 
                m_wantedState = WantedState::STAND_BY; // ignoring the command
            break;

        case WantedState::PREPARE_SHOOT:
            m_systemState = SystemState::RAMPING_TO_SHOOT;
            break;

        case WantedState::PREPARE_TO_KEEP_ALL:
            m_systemState = SystemState::SOON_MINE;
            break;

        case WantedState::RETRACT_HOOD:
            m_systemState = SystemState::RETRACTING_HOOD;
            break;

        case WantedState::KEEP_ALL_FOR_YOU:
            if (m_systemState != SystemState::THATS_ALL_MINE)
            {
                m_systemState = SystemState::SOON_MINE;
            }
            break;

        default:
            DEBUG_ASSERT(false,"Shooter : unknown wanted state used");
            break;
    }

    switch (m_systemState) // Change System State
    {
        case SystemState::IDLE:
        case SystemState::RESTING:
        case SystemState::SHOOTING_BACKWARD:
            break;

        case SystemState::RAMPING_TO_SHOOT:
        case SystemState::AT_SHOOT_SPEED:
            if(m_flywheelControlMode == FlywheelConstants::MainControlMode || m_hoodControlMode == HoodConstants::MainControlMode)
            {
                m_hoodTargetPos = m_pShootParameters->hoodAngle;
                m_flywheelTargetSpeed = m_pShootParameters->flywheelSpeed;
            }
            else
            {
                m_hoodTargetPos = HoodConstants::Position::AGAINST_HUB;
                m_flywheelTargetSpeed = FlywheelConstants::Speed::AGAINST_HUB;
            }

            if ((! IS_IN_RANGE(flywheelInputs.shooterVelocity,m_flywheelTargetSpeed,FlywheelConstants::Speed::TOLERANCE)
                || (! IS_IN_RANGE(hoodInputs.hoodAngle, m_hoodTargetPos, HoodConstants::Position::TOLERANCE)))
                && m_wantedState != WantedState::PREPARE_SHOOT)
                m_systemState = SystemState::RAMPING_TO_SHOOT;
            else
                m_systemState = SystemState::AT_SHOOT_SPEED;
            break; 

        case SystemState::RAMPING_BACKWARD:
            m_flywheelTargetSpeed = FlywheelConstants::Speed::BACKWARD;
            m_hoodTargetPos = HoodConstants::Position::MIN;

            if (IS_IN_RANGE(flywheelInputs.shooterVelocity,m_flywheelTargetSpeed,FlywheelConstants::Speed::TOLERANCE)
                && IS_IN_RANGE(hoodInputs.hoodAngle, m_hoodTargetPos, HoodConstants::Position::TOLERANCE))
                    m_systemState = SystemState::SHOOTING_BACKWARD;
            break;
        
        case SystemState::SLOWING_DOWN:
            m_flywheelTargetSpeed = FlywheelConstants::Speed::REST;
            m_hoodTargetPos = HoodConstants::Position::MIN;

            if (IS_IN_RANGE(flywheelInputs.shooterVelocity,m_flywheelTargetSpeed,FlywheelConstants::Speed::TOLERANCE)
                && IS_IN_RANGE(hoodInputs.hoodAngle, m_hoodTargetPos, HoodConstants::Position::TOLERANCE))
                    m_systemState = SystemState::RESTING;
            break;

        case SystemState::SOON_MINE:
        case SystemState::THATS_ALL_MINE:
            if(m_flywheelControlMode == FlywheelConstants::MainControlMode || m_hoodControlMode == HoodConstants::MainControlMode)
            {
                m_hoodTargetPos = m_pShootParameters->hoodAngle;
                m_flywheelTargetSpeed = m_pShootParameters->flywheelSpeed;
            }
            else
            {
                m_hoodTargetPos = HoodConstants::Position::TO_FAR_AWAY;
                m_flywheelTargetSpeed = FlywheelConstants::Speed::AGAINST_HUB;
            }

            if ((! IS_IN_RANGE(flywheelInputs.shooterVelocity,m_flywheelTargetSpeed,FlywheelConstants::Speed::TOLERANCE)
                || (! IS_IN_RANGE(hoodInputs.hoodAngle, m_hoodTargetPos, HoodConstants::Position::TOLERANCE)))
                && m_wantedState != WantedState::PREPARE_TO_KEEP_ALL)
                m_systemState = SystemState::SOON_MINE;
            else
                m_systemState = SystemState::THATS_ALL_MINE;
            break; 

        case SystemState::RETRACTING_HOOD:
            if (IS_IN_RANGE(hoodInputs.hoodAngle, 0.0, HoodConstants::Position::TOLERANCE))
            {
                m_systemState = SystemState::SLOWING_DOWN;
                m_wantedState = WantedState::STOP;
                m_currentWantedState = m_wantedState;
            }
            break;


        default:
            DEBUG_ASSERT(false, "shooter : impossible state");
            break; //end of default
    }
}