#pragma once

#include <functional>
#include <string>

class FeedForwardModel {
public :
  FeedForwardModel();
  FeedForwardModel(const double kS, const double kV, const double kA);

  /**
   * @brief Sets the Feedforward gains for the controller.
   * 
   * @param kS The static friction gain, representing the voltage needed to overcome static friction and start moving the mechanism.
   * @param kV The velocity gain, representing the voltage needed to maintain a certain velocity against viscous friction.
   * @param kA The acceleration gain, representing the voltage needed to achieve a certain acceleration, compensating for inertia.
   * @param kG An optional gravity gain, representing the voltage needed to counteract the effects of gravity on the mechanism. 
   * This is particularly useful for mechanisms like arms or elevators that are affected by gravity. 
   * The default value is a lambda function that returns 0.0, indicating no gravity compensation.
   */
  void SetGains(const double kS, const double kV, const double kA, const std::function<double(double)>& kG = [](double) { return 0.0; });
  /**
   * Sets the output limits for the Feedforward controller.
   *
   * @param min The minimum output value that the Feedforward controller can produce.
   * @param max The maximum output value that the Feedforward controller can produce.
   */
  void SetOutputLimits(const double min, const double max);

  double GetKS() const;
  double GetKV() const;
  double GetKA() const;
  double GetKG(const double position) const;
  /**
   * @brief Retrieves the current state of the Feedforward model as a formatted string.
   * 
   * @return A string containing the formatted state of the Feedforward model.
   */
  std::string GetState() const;

  /**
   * Calculates the Feedforward model output.
   *    
   * @param position The desired position of the mechanism.
   *                 Units: meters (m) or radians (rad). Be consistent with kV/kA units.
   * @param velocity The desired velocity of the mechanism.
   *                 Units: meters per second (m/s) or radians per second (rad/s).
   * @param acceleration The desired acceleration of the mechanism.
   *                     Units: meters per second squared (m/s^2) or radians per second squared (rad/s^2).
   * @return The Feedforward output value (units: volts, V).
   */
  double Calculate(const double position, const double velocity, const double acceleration);
private:

  double m_kS;  // static friction gain (starting threshold) (units: volts, V)
  double m_kV;  // velocity gain (viscous friction) (units: volts per velocity, e.g., V/(m/s) or V/(rad/s))
  double m_kA;  // acceleration gain (inertia compensation) (units: volts per acceleration, e.g., V/(m/s^2) or V/(rad/s^2))
  std::function<double(double)> m_kG; // gravity gain (for mechanisms affected by gravity, like arms or elevators)

  double m_outputMin{-1.0};   // Min output value
  double m_outputMax{1.0};    // Max output value

  double m_output{0.0};              // Output of the Feedforward controller.
}; 