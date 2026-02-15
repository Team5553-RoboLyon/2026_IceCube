// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include "ClimberConstants.h"
#include "ClimberIO.h"
#include "Constants.h"

#include "LyonLib/control/pidRBL.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

class ClimberSubsystem : public frc2::SubsystemBase {
  public:
    ClimberSubsystem(ClimberIO *pIO);

    enum class WantedState 
    {
      STAND_BY = 0,         // no wanted state scheduled.
      STOWED = 1,           // Be safe inside frame perimeter (match default)
      ARMED_TO_CLIMB = 2,   // Ready to climb
      CLIMBED = 3,          // Robot should be supported by the climber
      INITIALIZATION = 4
    };
    enum class SystemState
    {
      IDLE = 0,
      //Ste ady states
      STOWED_HOME = 1,      // Fully retracted, safe for match play
      ARMED = 2,            // Extended to pre-climb height
      CLIMBED_LOCKED = 3,   // robot hanging
      //Transition state
      EXTENDING_TO_ARMED = 4,
      RETRACTING_TO_HOME = 5,
      CLIMBING = 6
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
      TunableValueLogger m_tunableVoltage{"climber/AppliedVoltage",0.0};
    // === Visualization ===
      frc::Mechanism2d m_mechanism{ClimberConstants::Settings::TOP_LIMIT, ClimberConstants::Settings::TOP_LIMIT};
      frc::MechanismRoot2d* m_root{m_mechanism.GetRoot("climber", ClimberConstants::Settings::TOP_LIMIT/2, ClimberConstants::Settings::BOTTOM_LIMIT)};
      frc::MechanismLigament2d* m_hammer{m_root->Append<frc::MechanismLigament2d>("Hammer", 1, 90_deg)};
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