// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "turretConstants.h"
#include "turretIO.h"
#include "Constants.h"
#include "subsystems/ShootParametersCalculator.h"

#include "LyonLib/control/RateLimiter.h"
#include "LyonLib/control/pidRBL.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

#include "frc/DriverStation.h"

class TurretSubsystem : public frc2::SubsystemBase {
  public:
    TurretSubsystem(TurretIO *pIO, ShootParameters *pShootParams);

    enum class WantedState 
    {
      STAND_BY = 0, // no wanted state scheduled. (It's all good man, it's all good !)
      FOLLOW_HUB = 1,
      POINT_AT_ALLIANCE_ZONE = 2,
      // PREPARE_EJECT = 3
      
    };
    enum class SystemState
    {
      IDLE = 0,
      //steady states
      INACTIVE = 1,
      ALIGNED_WITH_HUB = 2,
      POINTING_AT_ALLIANCE_ZONE = 3,
      READY_TO_EJECT = 4,
      //transition states
      ALIGNING_WITH_HUB = 5,
      ALIGNING_WITH_ALLIANCE_ZONE = 6,
      SPINNING_TO_EJECT = 7,
    };
    void SetWantedState(const WantedState wantedState);
    SystemState GetSystemState();
    void SetControlMode(const ControlMode mode);
    ControlMode GetControlMode();
    void ToggleControlMode();

    bool IsResting();
    bool IsInitialized() { return m_isInitialized; }
    void SetManualControlInput(const double value);


    void Periodic() override;
  private:
    // === Hardware & IO Interfaces ===
      TurretIO *m_pTurretIO;
      TurretIOInputs inputs;
      // AprilTagPhotonCamera m_turretCamera{new photon::PhotonCamera{TurretConstants::TurretCamera::NAME}};
      ShootParameters* m_pShootParams;
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_controlMode = TurretConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_TurretPIDController;
    // === Control Inputs / Outputs ===
      double m_output{0.0};
      double m_manualControlInput{0.0};
      double m_timestamp{0.0};
      double m_targetPos{0.0};
      double m_highestHallEffectSensorValue{0.0};
      TunableValueLogger m_tunableRobotOrientation{"Tunable robot orientation", 0.0};
      TunableValueLogger m_tunableKP{"Tunable KP", TurretConstants::Gains::POSITION_DUTYCYCLE_PID::KP};
    // === Status Flags ===
      bool m_isInitialized = true;
      // bool m_isInBlueAlliance = true;
    // === System Alerts ===
      Alert m_motorDisconnected{"Turret Motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_motorHot{"Turret Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_motorOverheating{"Turret Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};