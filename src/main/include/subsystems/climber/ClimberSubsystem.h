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
      STAND_BY,         // no wanted state scheduled.
      STOWED,           // Be safe inside frame perimeter (match default)
      ARMED_TO_CLIMB,   // Ready to climb (extended but not loaded)
      CLIMBED,          // Robot should be supported by the climber
      RELEASED_CLIMB,    // Robot should be off the bar (declimbed)
      INITIALIZATION
    };
    enum class SystemState
    {
      IDLE,
      //Ste ady states
      STOWED_HOME,      // Fully retracted, safe for match play
      ARMED,            // Extended to pre-climb height
      CLIMBED_LOCKED,   // robot hanging
      //Transition state
      EXTENDING_TO_ARMED,
      RETRACTING_TO_HOME,
      CLIMBING
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
      bool m_isEncoderAlreadyReset = false; // Flag to prevent multiple encoder resets when hitting the bottom limit switch
    // === System Alerts ===
      Alert m_climberMotorDisconnected{"Climber Motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_climberMotorHot{"Climber Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_climberMotorOverheating{"Climber Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};