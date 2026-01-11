// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "shooterConstants.h"
#include "shooterIOLogger.h"
#include "shooterIO.h"
#include "Constants.h"

#include "LyonLib/control/RateLimiter.h"
#include "LyonLib/control/pidRBL.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

class ShooterSubsystem : public frc2::SubsystemBase {
  public:
    ShooterSubsystem(ShooterIO *pIO);

    enum class WantedState 
    {
      STAND_BY, // no wanted state scheduled. (It's all good man, it's all good !)
      SHOOT,
      STOP,
      REVERSE
    };
    enum class SystemState
    {
      IDLE,
      SHOOTING,
      REST,
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
      ShooterIO *m_pShooterIO;
      ShooterIOInputs inputs;
      ShooterIOLogger m_logger{frc::DataLogManager::GetLog(), "/Shooter"};
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_controlMode = ShooterConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_ShooterPIDController;
    // === Control Inputs / Outputs ===
      double m_output{0.0};
      double m_manualControlInput{0.0};
      double m_timestamp{0.0};
      double m_targetVelocity{0.0}; //RPM
      TunableValueLogger m_tunableVelocityLogger{"/Shooter", ShooterConstants::Specifications::LeftMotor_FREE_SPEED}; //RPM
    // === Status Flags ===
      bool m_isInitialized = true;
    // === System Alerts ===
      Alert m_LeftMotorDisconnected{"Shooter LeftMotor: Disconnected", Alert::AlertType::ERROR};
      Alert m_LeftMotorHot{"Shooter LeftMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_LeftMotorOverheating{"Shooter LeftMotor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_RightMotorDisconnected{"Shooter RightMotor: Disconnected", Alert::AlertType::ERROR};
      Alert m_RightMotorHot{"Shooter RightMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_RightMotorOverheating{"Shooter RightMotor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};