// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "intakeConstants.h"
#include "intakeIOLogger.h"
#include "intakeIO.h"
#include "Constants.h"

#include "LyonLib/control/RateLimiter.h"
#include "LyonLib/control/pidRBL.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

class IntakeSubsystem : public frc2::SubsystemBase {
  public:
    IntakeSubsystem(IntakeIO *pIO);

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
    void ResetEncoder();
    // void SetManualControlInput(const double value);
    // void SetManualControlInput(const std::function<double()> Axis); //temporary


    void Periodic() override;
  private:
    // === Hardware & IO Interfaces ===
      IntakeIO *m_pIntakeIO;
      IntakeIOInputs inputs;
      IntakeIOLogger m_logger{frc::DataLogManager::GetLog(), "/Intake"};
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_controlMode = IntakeConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_IntakePIDController;
    // === Control Inputs / Outputs ===
      double m_intakeOutput{0.0};
      double m_pivotOutput{0.0};
      double m_manualControlInput{0.0};
      double m_timestamp{0.0};
    // std::function<double()> m_fxAxis; //temporary
      TunableValueLogger m_tunablePivotVoltageLogger{"Intake/PivotVoltage", 0.0}; //RPM
      TunableValueLogger m_tunableIntakeVoltageLogger{"Intake/IntakeVoltage", 0.0}; //RPM
    // === Status Flags ===
      bool m_isInitialized = true;
    // === System Alerts ===
      Alert m_intakeMotorDisconnected{"Intake motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_intakeMotorHot{"Intake motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_intakeMotorOverheating{"Intake motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_pivotMotorDisconnected{"Pivot motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_pivotMotorHot{"Pivot motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_pivotMotorOverheating{"Pivot motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};