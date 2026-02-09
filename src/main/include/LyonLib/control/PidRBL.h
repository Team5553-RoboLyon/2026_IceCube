/*******************************************************************************
 * 
 * File        : PidRBL.h (v4.1)
 * Library     : LyonLib (from 2026)
 * Description : Advanced PID controller class implementing 
 *               Proportional-Integral-Derivative control. Supports real-time 
 *               update, input/output clamping, and continuous inputs (e.g. angle 
 *               wrap-around).
 * 
 * Authors     : Gaspard (2023), last update by AKA (2026) 
 *                                and inspired by Team 1678
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <string>
#define THEORETICAL_DT (0.02)

class PidRBL {
public :
  PidRBL();
  PidRBL(const double kp, const double ki, const double kd);


  /**
   * @brief Sets the PID (Proportional, Integral, Derivative) gains for the controller.
   * 
   * @note This function calls Reset() internally to clear the previous state of the controller.
   * 
   * @param kp The proportional gain, which determines the reaction to the current error.
   * @param ki The integral gain, which determines the reaction based on the accumulation of past errors.
   * @param kd The derivative gain, which determines the reaction based on the rate of change of the error.
   */
  void SetGains(const double kp, const double ki, const double kd);
  /**
   * @brief Sets the desired setpoint for the PID controller, clamping it within the allowed input range.
   * 
   * @param setpoint The desired setpoint value for the PID controller.
   */
  void SetSetpoint(const double setpoint);
  /**
   * Sets the output limits for the PID controller.
   *
   * @param min The minimum output value that the PID controller can produce.
   * @param max The maximum output value that the PID controller can produce.
   */
  void SetOutputLimits(const double min, const double max);
  /**
   * @brief Sets the input limits (acceptable range for setpoint values) for the PID controller.
   * 
   * @param min The minimum allowable input value.
   * @param max The maximum allowable input value.
   */
  void SetInputLimits(const double min, const double max);
  /**
   * @brief Sets the input limits for the PID controller.
   * 
   * This function activates or deactivates the input limits for the PID controller
   * based on the provided boolean parameter. When input limits are active, the 
   * controller will constrain the input values within a predefined range.
   * 
   * @param isActive A boolean value indicating whether input limits should be active.
   */
  void SetInputLimits(const bool isActive);
  /**
   * @brief Sets whether the PID controller should handle inputs as continuous.
   * 
   * When enabled, the controller will treat the input range as circular, allowing
   * for seamless transitions between the minimum and maximum values.
   * 
   * @param isContinuous A boolean indicating whether circular input handling is enabled.
   */
  void SetContinuous(const bool isContinuous);

  double GetKP() const;
  double GetKI() const;
  double GetKD() const;
  double GetError() const;
  double GetSetpoint() const;
  /**
   * @brief Retrieves the current state of the PID controller as a formatted string.
   * 
   * @return A string containing the formatted state of the PID controller.
   */
  std::string GetState() const;

    /**
   * Calculates the PID output using theoretical delta time (0.02s in FRC).
   *    
   * @param measurement The current value of the system being controlled.
   * @return The PID output value.
   */
  double Calculate(const double measurement);
  /**
   * Calculates the PID output using theoretical delta time (0.02s in FRC).
   *    
   * @param setpoint The desired target value for the system.
   * @param measurement The current value of the system being controlled.
   * @return The PID output value.
   */
  double Calculate(const double setpoint, const double measurement);
    /**
   * Calculates the PID output using real timestamp.
   *
   * @param measurement The current value of the system being controlled.
   * @param timestamp The current time in seconds, used for real-time calculations.
   * @return The PID output value.
   */
  double CalculateWithRealTime(const double measurement, const double timestamp);
  /**
   * Calculates the PID output using real timestamp.
   *
   * @param setpoint The desired target value for the system.
   * @param measurement The current value of the system being controlled.
   * @param timestamp The current time in seconds, used for real-time calculations.
   * @return The PID output value.
   */
  double CalculateWithRealTime(const double setpoint, const double measurement, const double timestamp);
   /**
   * @brief Resets the PID controller state.
   * 
   * This function adjusts the setpoint to match the current measurement.
   * It also resets the previous error, current error, and output to zero.
   * Additionally, it clears the integrative component of the PID controller.
   */
  void Reset();
  /**
   * @brief Resets the PID controller state and updates the last timestamp.
   * 
   * This method resets the internal state of the PID controller and sets
   * the last timestamp to the provided value. Also it calls Reset() to clear the previous state.
   * 
   * @param timestamp The current timestamp to set as the last timestamp.
   */
  void Reset(const double timestamp);
  /**
   * @brief Resets the integrative term accumulator to zero.
   * 
   * This function clears the I-term accumulator to prevent integrative windup
   * in the PID controller. It does not affect other components of the PID
   * controller.
   */
  void ResetIntegrative();
private:

  double m_kp;  // factor for Proportional gain
  double m_ki;  // factor for Integral gain
  double m_kd;  // factor for Derivative gain
  double m_kaw{0.0}; // Anti-windup back-calculation gain

  double m_outputMin{-1.0};   // Min output value
  double m_outputMax{1.0};    // Max output value
  double m_inputMin{0.0};    // Min setpoint value allowed
  double m_inputMax{0.0};    // Max setpoint value allowed

  bool m_isContinuous{false}; // do the endpoints wrap around?
  bool m_isInputLimitsActive{false}; // When set to true, the inputs will be constrained within specified limits.

  double m_previousMeasurement{0.0};  // the prior measurement for derivative calculation
  double m_integrative{0.0};     // Total accumulated error for integral term
  double m_setpoint{0.0};       // Desired setpoint
  double m_currentError{0.0};        // Error between the setpoint and the measurement

  double m_output{0.0};              // Output of the PID controller.
  
  double m_lastTimestamp{0.0};  // Last time the PID controller was updated
  double m_dt{THEORETICAL_DT}; // Time step for the PID controller, default in  FRC is 20ms
}; 