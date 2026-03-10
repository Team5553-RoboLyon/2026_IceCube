// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "hood/HoodConstants.h"
#include "flywheel/FlywheelConstants.h"
#include "hood/HoodIO.h"
#include "flywheel/FlywheelIO.h"
#include "Constants.h"
#include "subsystems/ShootParametersCalculator.h"

#include "LyonLib/control/RateLimiter.h"
#include "LyonLib/control/pidRBL.h"
#include "LyonLib/control/FeedforwardModel.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

class ShooterSubsystem : public frc2::SubsystemBase {
  public:
    ShooterSubsystem(FlywheelIO *pFlywheelIO, HoodIO *pHoodIO, ShootParameters* pShootParams);

    enum class WantedState 
    {
      STAND_BY = 0, // no wanted state scheduled. (It's all good man, it's all good !)
      SHOOT_TO_HUB = 1,
      // FEED_ALLY = 2,
      STOP = 3,
      // REVERSE = 4,
      PREPARE_SHOOT = 5,
      PREPARE_TO_KEEP_ALL = 6,
      RETRACT_HOOD = 7,
      KEEP_ALL_FOR_YOU = 8
    };

    enum class SystemState
    {
      IDLE = 0,
      //Steady states
      SHOOTING_TO_HUB = 1,
      // READY_TO_FEED = 2,
      RESTING = 3,
      // SHOOTING_BACKWARD = 4,
      THATS_ALL_MINE = 5,
      //Transition states
      RAMPING_TO_SHOOT = 6,
      // RAMPING_TO_FEED = 7,
      // RAMPING_BACKWARD = 8,
      SLOWING_DOWN = 9,
      SOON_MINE = 10,
      RETRACTING_HOOD = 11
    };

    void SetWantedState(const WantedState wantedState);
    SystemState GetSystemState();

    void SetControlMode(const ControlMode flywheelMode, const ControlMode hoodMode);
    void SetFlywheelControlMode(const ControlMode mode);
    void SetHoodControlMode(const ControlMode mode);
    ControlMode GetFlywheelControlMode();
    ControlMode GetHoodControlMode();
    void ToggleFlywheelControlMode();
    void ToggleHoodControlMode();

    bool IsHoodRetract();

    bool IsResting();
    bool IsInitialized() { return m_isInitialized; }
    void SetManualControlInput(const double value);


    void Periodic() override;
  private:
    // === Hardware & IO Interfaces ===
      FlywheelIO *m_pFlywheelIO;
      HoodIO *m_pHoodIO;
      FlywheelIOInputs flywheelInputs;
      HoodIOInputs hoodInputs;
      ShootParameters *m_pShootParameters;
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_flywheelControlMode = FlywheelConstants::MainControlMode;
      ControlMode m_hoodControlMode = HoodConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_flywheelPIDController;
      PidRBL m_hoodPIDController;
      FeedForwardModel m_flywheelFeedforward;
    // === Control Inputs / Outputs ===
      double m_manualControlInput{0.0};
      double m_timestamp{0.0};
      units::volt_t m_flywheelOutput{0.0};
      units::volt_t m_hoodOutput{0.0};

      double m_flywheelTargetSpeed{0.0};
      double m_hoodTargetPos{0.0};

    // === Status Flags ===
      bool m_isInitialized = true;
    // === System Alerts ===
      Alert m_leftMotorDisconnected{"Shooter LeftMotor: Disconnected", Alert::AlertType::ERROR};
      Alert m_leftMotorHot{"Shooter LeftMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_leftMotorOverheating{"Shooter LeftMotor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_rightMotorDisconnected{"Shooter RightMotor: Disconnected", Alert::AlertType::ERROR};
      Alert m_rightMotorHot{"Shooter RightMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_rightMotorOverheating{"Shooter RightMotor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_hoodMotorDisconnected{"Shooter HoodMotor: Disconnected", Alert::AlertType::ERROR};
      Alert m_hoodMotorHot{"Shooter HoodMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_hoodMotorOverheating{"Shooter HoodMotor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};