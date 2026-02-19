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
        // EVACUATE_SHOOTER = 3,
        // EJECT_FROM_INTAKE = 4,
        STOP_SHOOT = 5,
        STOP_INTAKE = 6,
        //Basic states : Do only one action
        REFUEL = 7,
        // FEED_ALLY = 8,
        // SHOOT_TO_ALLIANCE_ZONE = 9,
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
        // SHOOT_AT_HUB_REFUELING = 19+1,
        // SHOOT_AT_ALLIANCE_ZONE_REFUELING = 20+1,
        // FEED_ALLY_REFUELING = 21+1,
        //Optional states : States that are not necessary for the functioning of the robot but prepare for future actions
        // PREPARE_SHOOT_TO_HUB = 22+1,
        // PREPARE_SHOOT_TO_ALLAICNE_ZONE = 23+1,
        // PREPARE_ALLY_FEEDING = 24+1,
        PREPARE_REFUEL = 26,
        RETRACT_INTAKE = 27,
    };

    enum class SystemSuperState
    {
        IDLE = 0,
        //steady states
        REFUELING = 1,
        SHOOTING_TO_HUB = 2,
        READY_TO_REFUEL = 3,
        INTAKE_SAFE = 4,
        READY_TO_CLIMB = 5,
        CLIMBED = 6,
        AT_HOME = 7,
        //transition states
        EXTENDING_INTAKE = 8,
        PREPARING_TO_SHOOT = 9,
        MOVING_INTAKE_TO_SAFE_POS = 10,
        PREPARING_CLIMB = 11,
        CLIMBING = 12,
        RETRACTING_INTAKE = 13,
        RETRACTING_CLIMBER = 14
    };

    void SetWantedSuperState(WantedSuperState wantedSuperState);
    SystemSuperState GetSystemSuperState();

    void SetAlliance(frc::DriverStation::Alliance alliance);

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