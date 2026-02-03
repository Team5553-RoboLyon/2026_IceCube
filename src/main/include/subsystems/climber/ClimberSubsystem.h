// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ClimberConstants.h"
#include "ClimberIOLogger.h"
#include "ClimberIO.h"
#include "Constants.h"

#include "LyonLib/control/RateLimiter.h"
#include "LyonLib/control/pidRBL.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

class ClimberSubsystem : public frc2::SubsystemBase {
  public:
    ClimberSubsystem(ClimberIO *pIO);

    enum class WantedState 
    {
      STAND_BY // no wanted state scheduled. (It's all good man, it's all good !)
    };
    enum class SystemState
    {
      IDLE
    };
    void SetWantedState(const WantedState wantedState);
    SystemState GetSystemState();
    void SetControlMode(const ControlMode mode);
    ControlMode GetControlMode();
    void ToggleControlMode();

    bool IsResting();
    bool IsInitialized() { return m_isInitialized; }
    void SetManualControlInput(const double value);

    void ResetEncoder();

    void Periodic() override;
  private:
    // === Hardware & IO Interfaces ===
      ClimberIO *m_pClimberIO;
      ClimberIOInputs inputs;
      ClimberIOLogger m_logger{frc::DataLogManager::GetLog(), "/Climber"};
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_controlMode = ClimberConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_ClimberPIDController;
    // === Control Inputs / Outputs ===
      double m_output{0.0};
      double m_manualControlInput{0.0};
      double m_timestamp{0.0};
      TunableValueLogger m_tunableVoltage{"Climber/AppliedVoltage",0.0};
    // === Status Flags ===
      bool m_isInitialized = true;
    // === System Alerts ===
      Alert m_climberMotorDisconnected{"Climber climberMotor: Disconnected", Alert::AlertType::ERROR};
      Alert m_climberMotorHot{"Climber climberMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_climberMotorOverheating{"Climber climberMotor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};