#pragma once

#include <frc2/command/SubsystemBase.h>

#include "subsystems/climber/ClimberSubsystem.h"
#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/shooter/ShooterSubsystem.h"
#include "subsystems/turret/TurretSubsystem.h"
#include "subsystems/ShootParametersCalculator.h"
#include "RobotState.h"

#include "LyonLib/logging/ComplexStructLogger.h"

#include "frc/geometry/Pose2d.h"

class Superstructure final : public frc2::SubsystemBase
{
    public:

    Superstructure(IntakeSubsystem *pIntake, 
                               IndexerSubsystem *pIndexer,
                               TurretSubsystem *pTurret,
                               ShooterSubsystem *pShooter,
                               ClimberSubsystem *pClimber,
                               ShootParameters *pShootParams,
                               RobotState* pRobotState);
    ~Superstructure() = default;

    enum class WantedSuperState
    {
        STAND_BY = 0,
        // INIT,
        //Priority states : If they have been used it's because we need to use them (mostly emergency states)
        PROTECT_INTAKE = 1,
        EVACUATE_SHOOTER = 2,
        STOP_SHOOT = 3,
        STOP_INTAKE = 4,
        RETRACT_HOOD = 5,
        //Basic states : Do only one action
        REFUEL = 6,
        // FEED_ALLY,
        SHOOT_TO_ALLIANCE_ZONE = 7,
        SHOOT_TO_HUB = 8,
        PREPARE_CLIMB = 9,
        RETRACT_CLIMBER = 10,
        CLIMB = 11,
        REJECT_FROM_INTAKE = 12,
        //Combining states : Combine two are more basic states
        // RETRACT_CLIMBER_REFUELING,
        // RETRACT_CLIMBER_SHOOTING_AT_HUB,
        // RETRACT_CLIMBER_FEEDING_ALLY,
        // PREPARE_CLIMB_REFUELING,
        // PREPARE_CLIMB_SHOOTING_AT_HUB,
        // PREPARE_CLIMB_FEEDING_ALLY,
        SHOOT_AT_HUB_REFUELING = 13,
        SHOOT_AT_ALLIANCE_ZONE_REFUELING = 14,
        // FEED_ALLY_REFUELING,
        //Optional states : States that are not necessary for the functioning of the robot but prepare for future actions
        PREPARE_SHOOT_TO_HUB = 15,
        PREPARE_SHOOT_TO_ALLIANCE_ZONE = 16,
        // PREPARE_ALLY_FEEDING,
        PREPARE_REFUEL = 17,
        RETRACT_INTAKE = 18,
    };

    enum class SystemSuperState
    {
        IDLE = 0,
        //steady states
        REFUELING = 1,
        SHOOTING_TO_ALLIANCE_ZONE = 2,
        SHOOTING_TO_HUB = 3,
        READY_TO_REFUEL = 4,
        INTAKE_SAFE = 5,
        CLIMBED = 6,
        AT_HOME = 7,
        SHOOTING_TO_HUB_WHILE_REFUELING = 8,
        SHOOTING_TO_ALLIANCE_ZONE_WHILE_REFUELING = 9,
        REJECTING_FROM_INTAKE = 10,
        //transition states
        EXTENDING_INTAKE = 11,
        PREPARING_ALLIANCE_ZONE_SHOOT = 12,
        PREPARING_TO_SHOOT = 13,
        MOVING_INTAKE_TO_SAFE_POS = 14,
        PREPARING_CLIMB = 15,
        CLIMBING = 16,
        RETRACTING_INTAKE = 17,
        RETRACTING_CLIMBER = 18,
        EVACUATING_SHOOTER = 19,
        PREPARING_TO_SHOOT_TO_HUB_WHILE_REFUELING = 20,
        PREPARING_TO_SHOOT_TO_ALLIANCE_ZONE_WHILE_REFUELING = 21,
        RETRACTING_HOOD = 22,
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

    void TogglePivotMantainPID();

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

    // === Output wanted states (used for subsystems with more security, ex : secure pos when approaching to trench)
    IntakeSubsystem::WantedState m_intakeWantedState;
    ShooterSubsystem::WantedState m_shooterWantedState;

    // === Calculator ===
    ShootParametersCalculator m_shootParameterCalculator;
    ShootParameters *m_pShootParameters;

    // === External Inputs ===
    RobotState *m_pRobotState;
    frc::Pose2d m_robotPos;
    StructLogger<frc::Pose2d> m_loggerPos{"/superstructurePos"};
    double m_timestamp{0.0};

    // === Internal methods ===
    void RunSuperStateMachine();
    bool IsRobotCloseToTrench();

    // StructLogger<frc::Pose2d> m_logger{"RobotPos"};
};