#include "subsystems/superstrucure/Superstructure.h"

#include "LyonLib/utils/TimerRBL.h"
#include "frc/smartdashboard/SmartDashboard.h"

Superstructure::Superstructure(IntakeSubsystem *pIntake, 
                               IndexerSubsystem *pIndexer,
                               TurretSubsystem *pTurret,
                               ShooterSubsystem *pShooter,
                               ClimberSubsystem *pClimber,
                               ShootParameters *pShootParams)
                               : m_pIntake(pIntake),
                                 m_pIndexer(pIndexer),
                                 m_pTurret(pTurret),
                                 m_pShooter(pShooter),
                                 m_pClimber(pClimber),
                                 m_pShootParameters(pShootParams)
{
    
}

void Superstructure::SetWantedSuperState(WantedSuperState wantedSuperState)
{
    m_wantedSuperState = wantedSuperState;
}

Superstructure::SystemSuperState Superstructure::GetSystemSuperState()
{
    return m_systemSuperState;
}

void Superstructure::SetAlliance()
{
    m_shootParameterCalculator.SetAlliance(frc::DriverStation::GetAlliance().value());
}

void Superstructure::EnableSubsystems()
{
    m_pIntake->SetControlMode(PivotConstants::MainControlMode, RollerConstants::MainControlMode);
    m_pIndexer->SetControlMode(IndexerConstants::MainControlMode);
    m_pTurret->SetControlMode(TurretConstants::MainControlMode);
    m_pShooter->SetControlMode(FlywheelConstants::MainControlMode, HoodConstants::MainControlMode);
    m_pClimber->SetControlMode(ClimberConstants::MainControlMode);
}

void Superstructure::DisableSubsystems()
{
    m_pIntake->SetControlMode(ControlMode::DISABLED, ControlMode::DISABLED);
    m_pIndexer->SetControlMode(ControlMode::DISABLED);
    m_pTurret->SetControlMode(ControlMode::DISABLED);
    m_pShooter->SetControlMode(ControlMode::DISABLED, ControlMode::DISABLED);
    m_pClimber->SetControlMode(ControlMode::DISABLED);
}

void Superstructure::ToggleIntakeControlMode()
{
    m_pIntake->TogglePivotControlMode();
    m_pIntake->ToggleRollerControlMode();
}

void Superstructure::ToggleIndexerControlMode()
{
    m_pIndexer->ToggleControlMode();
}

void Superstructure::ToggleTurretControlMode()
{
    m_pTurret->ToggleControlMode();
}

void Superstructure::ToggleShooterControlMode()
{
    m_pShooter->ToggleHoodControlMode();
    m_pShooter->ToggleFlywheelControlMode();
}

void Superstructure::ToggleClimberControlMode()
{
    m_pClimber->ToggleControlMode();
}

void Superstructure::Periodic()
{
    m_currentWantedSuperState = m_wantedSuperState;
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();

    if (m_currentWantedSuperState == WantedSuperState::SHOOT_TO_HUB
        || m_currentWantedSuperState == WantedSuperState::PREPARE_SHOOT_TO_HUB)
    {
        m_shootParameterCalculator.CalculateHubNewParameters(*m_pShootParameters, m_robotPos, m_timestamp);
    }
    else if (m_currentWantedSuperState == WantedSuperState::SHOOT_TO_ALLIANCE_ZONE
             || m_currentWantedSuperState == WantedSuperState::PREPARE_SHOOT_TO_ALLIANCE_ZONE)
    {
        m_shootParameterCalculator.CalculateAllianceZoneNewParameters(*m_pShootParameters, m_robotPos, m_timestamp);
    }
    else
    {
        m_shootParameterCalculator.SetRobotPos(m_robotPos, m_timestamp);
    }

    RunSuperStateMachine();

    switch(m_systemSuperState)
    {
        case SystemSuperState::IDLE:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::STAND_BY);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::REFUELING:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::REFUEL);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::BECOME_AN_INDEXER);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::FEED_SHOOTER);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::KEEP_ALL_FOR_YOU);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::SHOOTING_TO_HUB:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::BECOME_AN_INDEXER);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::FEED_SHOOTER);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::FOLLOW_HUB);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::SHOOT_TO_HUB);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::READY_TO_REFUEL:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::EXTEND);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::INTAKE_SAFE:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::PROTECT_YOURSELF_AGAINST_EVIL_PILOT);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::READY_TO_CLIMB:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::RETURN_AT_HOME);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::ARMED_TO_CLIMB);
            break;

        case SystemSuperState::CLIMBED:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::RETURN_AT_HOME);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::CLIMBED);
            break;

        case SystemSuperState::AT_HOME:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::RETURN_AT_HOME);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::EXTENDING_INTAKE:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::EXTEND);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::PREPARING_ALLIANCE_ZONE_SHOOT:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::STAND_BY);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::PREPARE_SHOOT);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::PREPARE_TO_KEEP_ALL);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::PREPARING_TO_SHOOT:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::STAND_BY);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::PREPARE_SHOOT);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::FOLLOW_HUB);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::PREPARE_HUB_SHOOTING);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::MOVING_INTAKE_TO_SAFE_POS:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::PROTECT_YOURSELF_AGAINST_EVIL_PILOT);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::PREPARING_CLIMB:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::RETURN_AT_HOME);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::ARMED_TO_CLIMB);
            break;

        case SystemSuperState::CLIMBING:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::RETURN_AT_HOME);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::CLIMBED);
            break;

        case SystemSuperState::RETRACTING_INTAKE:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::RETURN_AT_HOME);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STAND_BY);
            break;

        case SystemSuperState::RETRACTING_CLIMBER:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::STAND_BY);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::STAND_BY);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::STOP);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::EVACUATING_SHOOTER:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::EXTEND);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::EVACUATE_SHOOTER);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::STAND_BY);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::REVERSE);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::SHOOTING_TO_HUB_WHILE_REFUELING:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::REFUEL);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::FEED_SHOOTER);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::FOLLOW_HUB);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::SHOOT_TO_HUB);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::PREPARING_TO_SHOOT_TO_HUB_WHILE_REFUELING:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::REFUEL);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::PREPARE_SHOOT);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::FOLLOW_HUB);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::SHOOT_TO_HUB);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::PREPARING_TO_SHOOT_TO_ALLIANCE_ZONE_WHILE_REFUELING:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::REFUEL);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::PREPARE_SHOOT);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::KEEP_ALL_FOR_YOU);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        case SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE_WHILE_REFUELING:
            m_pIntake->SetWantedState(IntakeSubsystem::WantedState::REFUEL);
            m_pIndexer->SetWantedState(IndexerSubsystem::WantedState::FEED_SHOOTER);
            m_pTurret->SetWantedState(TurretSubsystem::WantedState::POINT_AT_ALLIANCE_ZONE);
            m_pShooter->SetWantedState(ShooterSubsystem::WantedState::KEEP_ALL_FOR_YOU);
            m_pClimber->SetWantedState(ClimberSubsystem::WantedState::STOWED);
            break;

        default:
            DEBUG_ASSERT(false,"Superstructure : unknwon system super state used");
            break;
    }

    frc::SmartDashboard::PutNumber("Superstructure/WantedSuperState", (int)m_wantedSuperState);
    frc::SmartDashboard::PutNumber("Superstructure/SystemSuperState", (int)m_systemSuperState);

}

void Superstructure::SetRobotPos(frc::Pose2d robotPos)
{
    m_robotPos = robotPos;
}

void Superstructure::RunSuperStateMachine()
{
    switch(m_currentWantedSuperState)
    {
        case WantedSuperState::STAND_BY:
            if (m_pIntake->IsOut())
            {
                if (m_pIntake->IsPivotMoving())
                {
                    m_systemSuperState = SystemSuperState::EXTENDING_INTAKE;
                }
                else
                {
                    m_systemSuperState = SystemSuperState::READY_TO_REFUEL;
                }
            }
            else
            {
                if (m_pIntake->IsPivotMoving())
                {
                    m_systemSuperState = SystemSuperState::RETRACTING_INTAKE;
                }
                else
                {
                    m_systemSuperState = SystemSuperState::AT_HOME;
                }
            }
            break;

        case WantedSuperState::PROTECT_INTAKE:
            if (m_systemSuperState != SystemSuperState::INTAKE_SAFE)
                m_systemSuperState = SystemSuperState::MOVING_INTAKE_TO_SAFE_POS;
            break;

        case WantedSuperState::STOP_SHOOT:
            if (m_systemSuperState == SystemSuperState::PREPARING_TO_SHOOT
                || m_systemSuperState == SystemSuperState::SHOOTING_TO_HUB)
                {
                    m_systemSuperState = SystemSuperState::IDLE;
                    m_wantedSuperState = WantedSuperState::STAND_BY;
                    m_currentWantedSuperState = m_wantedSuperState;
                }
            break;

        case WantedSuperState::STOP_INTAKE:
            if (m_systemSuperState == SystemSuperState::REFUELING)
            {
                m_systemSuperState = SystemSuperState::IDLE;
            }
            break;

        case WantedSuperState::REFUEL:
            if (m_systemSuperState != SystemSuperState::REFUELING)
            {
                m_systemSuperState = SystemSuperState::REFUELING;
            }
            break;

        case WantedSuperState::SHOOT_TO_ALLIANCE_ZONE:
            if (m_systemSuperState != SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE)
            {
                m_systemSuperState = SystemSuperState::PREPARING_ALLIANCE_ZONE_SHOOT;
            }
            break;

        case WantedSuperState::SHOOT_TO_HUB:
            if (m_systemSuperState != SystemSuperState::SHOOTING_TO_HUB)
            {
                m_systemSuperState = SystemSuperState::SHOOTING_TO_HUB;
            }
            break;

        case WantedSuperState::PREPARE_CLIMB:
            if (m_systemSuperState != SystemSuperState::READY_TO_CLIMB)
            {
                m_systemSuperState = SystemSuperState::PREPARING_CLIMB;
            }
            break;

        case WantedSuperState::RETRACT_CLIMBER:
            if (m_systemSuperState != SystemSuperState::RETRACTING_CLIMBER)
            {
                m_systemSuperState = SystemSuperState::RETRACTING_CLIMBER;
            }
            break;

        case WantedSuperState::CLIMB:
            if(m_pClimber->GetSystemState() != ClimberSubsystem::SystemState::ARMED
               || m_pClimber->GetSystemState() != ClimberSubsystem::SystemState::CLIMBING
               || m_pClimber->GetSystemState() != ClimberSubsystem::SystemState::CLIMBED_LOCKED)
            {
                m_systemSuperState = SystemSuperState::PREPARING_CLIMB;
            }
            else
            {
                m_systemSuperState = SystemSuperState::READY_TO_CLIMB;
            }
            break;

        case WantedSuperState::PREPARE_SHOOT_TO_HUB:
            if(m_systemSuperState != SystemSuperState::PREPARING_TO_SHOOT)
            {
                m_systemSuperState = SystemSuperState::PREPARING_TO_SHOOT;
            }
            break;

        case WantedSuperState::PREPARE_SHOOT_TO_ALLIANCE_ZONE:
            if(m_systemSuperState != SystemSuperState::PREPARING_ALLIANCE_ZONE_SHOOT)
            {
                m_systemSuperState = SystemSuperState::PREPARING_ALLIANCE_ZONE_SHOOT;
            }
            break;
            

        case WantedSuperState::PREPARE_REFUEL:
            if (m_systemSuperState != SystemSuperState::READY_TO_REFUEL)
                m_systemSuperState = SystemSuperState::EXTENDING_INTAKE;
            break;

        case WantedSuperState::RETRACT_INTAKE:
            if (m_systemSuperState != SystemSuperState::AT_HOME)
                m_systemSuperState = SystemSuperState::RETRACTING_INTAKE;
            else 
            {
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = m_wantedSuperState;
            }
            break;

        case WantedSuperState::EVACUATE_SHOOTER:
            m_systemSuperState = SystemSuperState::EVACUATING_SHOOTER;
            break;

        case WantedSuperState::SHOOT_AT_HUB_REFUELING:
            if(m_systemSuperState != SystemSuperState::SHOOTING_TO_HUB_WHILE_REFUELING)
            {
                m_systemSuperState = SystemSuperState::PREPARING_TO_SHOOT_TO_HUB_WHILE_REFUELING;
            }
            break;

        case WantedSuperState::SHOOT_AT_ALLIANCE_ZONE_REFUELING:
            if(m_systemSuperState != SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE_WHILE_REFUELING)
            {
                m_systemSuperState = SystemSuperState::PREPARING_TO_SHOOT_TO_ALLIANCE_ZONE_WHILE_REFUELING;
            }
            break;

        default:
            DEBUG_ASSERT(false, "Superstructure : unknown wanted super state used");
            break;
    }


    switch (m_systemSuperState)
    {
        case SystemSuperState::IDLE:
            if(m_pIntake->IsOut())
            {
                if (m_pIntake->IsPivotMoving())
                {
                    m_systemSuperState = SystemSuperState::EXTENDING_INTAKE;
                }
                else
                {
                    m_systemSuperState = SystemSuperState::READY_TO_REFUEL;
                }  
            }
            else
            {
                if (m_pIntake->IsPivotMoving())
                {
                    m_systemSuperState = SystemSuperState::RETRACTING_INTAKE;
                }
                else
                {
                    m_systemSuperState = SystemSuperState::AT_HOME;
                }   
            }
            break;

        case SystemSuperState::REFUELING:
        case SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE:
        case SystemSuperState::SHOOTING_TO_HUB:
        case SystemSuperState::READY_TO_REFUEL:
        case SystemSuperState::INTAKE_SAFE:
        case SystemSuperState::READY_TO_CLIMB:
        case SystemSuperState::CLIMBED:
        case SystemSuperState::AT_HOME:
            break;

        case SystemSuperState::EXTENDING_INTAKE:
            if (m_pIntake->GetSystemState() == IntakeSubsystem::SystemState::CHILLING_OUT)
            {
                if (m_wantedSuperState != WantedSuperState::REFUEL)
                {
                    m_wantedSuperState = WantedSuperState::STAND_BY;
                    m_currentWantedSuperState = m_wantedSuperState;
                }
                m_systemSuperState = SystemSuperState::READY_TO_REFUEL;
            }
            break;

        case SystemSuperState::PREPARING_ALLIANCE_ZONE_SHOOT:
            if(m_pShooter->GetSystemState() == ShooterSubsystem::SystemState::THATS_ALL_MINE
               && m_pTurret->GetSystemState() == TurretSubsystem::SystemState::POINTING_AT_ALLIANCE_ZONE
               && m_wantedSuperState == WantedSuperState::SHOOT_TO_ALLIANCE_ZONE)
            {
                m_systemSuperState = SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE;
            }
            break;


        case SystemSuperState::PREPARING_TO_SHOOT:
            if (m_pShooter->GetSystemState() == ShooterSubsystem::SystemState::AT_SHOOT_SPEED
                && m_pTurret->GetSystemState() == TurretSubsystem::SystemState::ALIGNED_WITH_HUB
                && m_wantedSuperState == WantedSuperState::SHOOT_TO_HUB)
            {
                m_systemSuperState = SystemSuperState::SHOOTING_TO_HUB;
            }
            break;

        case SystemSuperState::MOVING_INTAKE_TO_SAFE_POS:
            if (m_pIntake->GetSystemState() == IntakeSubsystem::SystemState::PROTECTED_AGAINST_EVIL_PILOT)
            {
                m_systemSuperState = SystemSuperState::INTAKE_SAFE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = m_wantedSuperState;
            }
            break;

        case SystemSuperState::PREPARING_CLIMB:
            if (m_pClimber->GetSystemState() == ClimberSubsystem::SystemState::ARMED)
            {
                m_systemSuperState = SystemSuperState::READY_TO_CLIMB;
                if (m_systemSuperState != SystemSuperState::CLIMBED)
                {
                    m_wantedSuperState = WantedSuperState::STAND_BY;
                    m_currentWantedSuperState = m_wantedSuperState;
                }
            }
            break;

        case SystemSuperState::CLIMBING:
            if(m_pClimber->GetSystemState() == ClimberSubsystem::SystemState::CLIMBED_LOCKED)
            {
                m_systemSuperState = SystemSuperState::CLIMBED;
            }
            break;

        case SystemSuperState::RETRACTING_INTAKE:
            if(m_pIntake->GetSystemState() == IntakeSubsystem::SystemState::STAYING_AT_HOME)
            {
                m_systemSuperState = SystemSuperState::AT_HOME;
                if (m_wantedSuperState != WantedSuperState::SHOOT_TO_HUB
                    && m_wantedSuperState != WantedSuperState::CLIMB
                    && m_wantedSuperState != WantedSuperState::PREPARE_CLIMB)
                {
                    m_wantedSuperState = WantedSuperState::STAND_BY;
                    m_currentWantedSuperState = m_wantedSuperState;
                }
            }
            break;

        case SystemSuperState::RETRACTING_CLIMBER:
            if(m_pClimber->GetSystemState() == ClimberSubsystem::SystemState::STOWED_HOME)
            {
                m_systemSuperState = SystemSuperState::IDLE;
            }
            break;

        case SystemSuperState::EVACUATING_SHOOTER:
            if(m_pIndexer->GetSystemState() == IndexerSubsystem::SystemState::SLEEPING &&
               (m_pShooter->GetSystemState() == ShooterSubsystem::SystemState::RAMPING_BACKWARD
                 || m_pShooter->GetSystemState() == ShooterSubsystem::SystemState::SHOOTING_BACKWARD))
                {
                    m_systemSuperState = SystemSuperState::IDLE;
                    m_wantedSuperState = WantedSuperState::STAND_BY;
                    m_currentWantedSuperState = m_wantedSuperState;
                }
                break;

        case SystemSuperState::SHOOTING_TO_HUB_WHILE_REFUELING:
        case SystemSuperState::PREPARING_TO_SHOOT_TO_HUB_WHILE_REFUELING:
            if(m_pShooter->GetSystemState() == ShooterSubsystem::SystemState::AT_SHOOT_SPEED
                                && m_pTurret->GetSystemState() == TurretSubsystem::SystemState::ALIGNED_WITH_HUB)
            {
                m_systemSuperState = SystemSuperState::SHOOTING_TO_HUB_WHILE_REFUELING;
            }
            else
            {
                m_systemSuperState = SystemSuperState::PREPARING_TO_SHOOT_TO_HUB_WHILE_REFUELING;
            }
            break;

        case SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE_WHILE_REFUELING:
        case SystemSuperState::PREPARING_TO_SHOOT_TO_ALLIANCE_ZONE_WHILE_REFUELING:
            if(m_pShooter->GetSystemState() == ShooterSubsystem::SystemState::THATS_ALL_MINE
                                && m_pTurret->GetSystemState() == TurretSubsystem::SystemState::POINTING_AT_ALLIANCE_ZONE)
            {
                m_systemSuperState = SystemSuperState::SHOOTING_TO_ALLIANCE_ZONE_WHILE_REFUELING;
            }
            else
            {
                m_systemSuperState = SystemSuperState::PREPARING_TO_SHOOT_TO_ALLIANCE_ZONE_WHILE_REFUELING;
            }
            break;

        default:
            DEBUG_ASSERT(false,"Superstrcture : unknown system super state");
            break;
    }
}