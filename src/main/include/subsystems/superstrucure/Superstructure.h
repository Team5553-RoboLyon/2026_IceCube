#pragma once

#include <frc2/command/SubsystemBase.h>

#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/shooter/ShooterSubsystem.h"
#include "subsystems/turret/TurretSubsystem.h"
#include "subsystems/ShootParametersCalculator.h"

#include "frc/geometry/Pose2d.h"

class Superstructure final : public frc2::SubsystemBase
{
    public:

    Superstructure(IntakeSubsystem *pIntake, 
                               IndexerSubsystem *pIndexer,
                               TurretSubsystem *pTurret,
                               ShooterSubsystem *pShooter,
                               ClimberSubsystem *pClimber,
                               ShootParameters *pShootParams);
    ~Superstructure() = default;

    enum class WantedSuperState
    {
        STAND_BY = 0,
        // INIT = 1,
        //Priority states : If they have been used it's because we need to use them (mostly emergency states)
        PROTECT_INTAKE = 2,
        EVACUATE_SHOOTER = 3,
        // EJECT_FROM_INTAKE = 4,
        STOP_SHOOT = 5,
        STOP_INTAKE = 6,
        //Basic states : Do only one action
        REFUEL = 7,
        // FEED_ALLY = 8,
        SHOOT_TO_ALLIANCE_ZONE = 9,
        SHOOT_TO_HUB = 10,
        PREPARE_CLIMB = 11,
        RETRACT_CLIMBER = 12,
        CLIMB = 13,
        //Combining states : Combine two are more basic states
        // RETRACT_CLIMBER_REFUELING = 13+1,
        // RETRACT_CLIMBER_SHOOTING_AT_HUB = 14+1,
        // RETRACT_CLIMBER_FEEDING_ALLY = 15+1,
        // PREPARE_CLIMB_REFUELING = 16+1,
        // PREPARE_CLIMB_SHOOTING_AT_HUB = 17+1,
        // PREPARE_CLIMB_FEEDING_ALLY = 18+1,
        SHOOT_AT_HUB_REFUELING = 20,
        SHOOT_AT_ALLIANCE_ZONE_REFUELING = 21,
        // FEED_ALLY_REFUELING = 21+1,
        //Optional states : States that are not necessary for the functioning of the robot but prepare for future actions
        PREPARE_SHOOT_TO_HUB = 23,
        PREPARE_SHOOT_TO_ALLIANCE_ZONE = 24,
        // PREPARE_ALLY_FEEDING = 24+1,
        PREPARE_REFUEL = 26,
        RETRACT_INTAKE = 27,
    };

    enum class SystemSuperState
    {
        IDLE = 0,
        //steady states
        REFUELING = 1,
        SHOOTING_TO_ALLIANCE_ZONE = 2,
        SHOOTING_TO_HUB = 2+1,
        READY_TO_REFUEL = 3+1,
        INTAKE_SAFE = 4+1,
        CLIMBED = 6+1,
        AT_HOME = 7+1,
        SHOOTING_TO_HUB_WHILE_REFUELING = 9,
        SHOOTING_TO_ALLIANCE_ZONE_WHILE_REFUELING,
        //transition states
        EXTENDING_INTAKE = 11,
        PREPARING_ALLIANCE_ZONE_SHOOT,
        PREPARING_TO_SHOOT,
        MOVING_INTAKE_TO_SAFE_POS,
        PREPARING_CLIMB,
        CLIMBING,
        RETRACTING_INTAKE,
        RETRACTING_CLIMBER,
        EVACUATING_SHOOTER,
        PREPARING_TO_SHOOT_TO_HUB_WHILE_REFUELING,
        PREPARING_TO_SHOOT_TO_ALLIANCE_ZONE_WHILE_REFUELING,
    };

    void SetWantedSuperState(WantedSuperState wantedSuperState);
    SystemSuperState GetSystemSuperState();

    void SetAlliance();

    void EnableSubsystems();
    void DisableSubsystems();

    void ToggleIntakeControlMode();
    void ToggleIndexerControlMode();
    void ToggleTurretControlMode();
    void ToggleShooterControlMode();
    void ToggleClimberControlMode();

    void Periodic() override;

    void SetRobotPos(frc::Pose2d robotPos);

    private:

    // === Subsystems ===
    IntakeSubsystem *m_pIntake;
    IndexerSubsystem *m_pIndexer;
    TurretSubsystem *m_pTurret;
    ShooterSubsystem *m_pShooter;
    ClimberSubsystem *m_pClimber;

    // === System SuperState ===
    WantedSuperState m_wantedSuperState = WantedSuperState::STAND_BY;
    WantedSuperState m_currentWantedSuperState = m_wantedSuperState;
    SystemSuperState m_systemSuperState = SystemSuperState::IDLE;
    bool m_IsInitialized = true;

    // === Calculator ===
    ShootParametersCalculator m_shootParameterCalculator;
    ShootParameters *m_pShootParameters;

    // === External Inputs ===
    frc::Pose2d m_robotPos;
    double m_timestamp{0.0};

    // === Internal methods ===
    void RunSuperStateMachine();
};