/*******************************************************************************
 * 
 * File        : XboxGamepad.h (v1.0)
 * Library     : LyonLib (from 2026_NoName)
 * Description : Handle input from Xbox One Gamepad connected to the Driver Station.
 * 
 * Authors     : VTT (2026), last update by VTT (2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include "frc/GenericHID.h"
#include "frc2/command/button/Trigger.h"

//WARNING: all functions are not tested, please report any bug you find

/**
 * Handle input from Xbox One Gamepad connected to the Driver Station.
 *
 * This class handles Xbox input that comes from the Driver Station. Each
 * time a value is requested the most recent value is returned. There is a
 * single class instance for each controller and the mapping of ports to
 * hardware buttons depends on the code in the Driver Station.
 */
class XboxGamepad : public frc::GenericHID{
 public:
    /**
     * Construct an instance of a controller.
     *
     * The controller index is the USB port on the Driver Station.
     * The threshold for the axis to be considered pressed as a button is set to 0.5.
     *
     * @param port The port on the Driver Station that the controller is plugged
     *             into (0-5).
     */
    explicit XboxGamepad(int port);

    /**
     * Construct an instance of a controller.
     *
     * The controller index is the USB port on the Driver Station.
     *
     * @param port The port on the Driver Station that the controller is plugged
     *             into (0-5).
     * @param threshold The threshold for the axis to be considered pressed as a button.
     */
    explicit XboxGamepad(int port, double threshold);

    ~XboxGamepad() override = default;

    XboxGamepad(XboxGamepad&&) = default;
    XboxGamepad& operator=(XboxGamepad&&) = default;

    /**
     * Get the X axis value of left side of the controller.
     * @return the axis value.
     */
    double GetLeftX() const;
    /**
     * Get the Y axis value of left side of the controller.
     * @return the axis value.
     */
    double GetLeftY() const;

    /**
     * Get the X axis value of right side of the controller.
     * @return the axis value.
     */
    double GetRightX() const;
    /**
     * Get the Y axis value of right side of the controller.
     * @return the axis value.
     */
    double GetRightY() const;

    /**
     * Get the left trigger 2 axis value of the controller. Note that this axis
     * is bound to the range of [0, 1].
     * @return the axis value.
     */
    double GetL2Axis() const;
    /**
     * Get the right trigger 2 axis value of the controller. Note that this axis
     * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
     * @return the axis value.
     */
    double GetR2Axis() const;

    /**
     * Read the value of the X button on the controller.
     * @return The state of the button.
     */
    bool GetXButton() const;
    /**
     * Whether the X button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetXButtonPressed();
    /**
     * Whether the X button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetXButtonReleased();

    /**
     * Read the value of the Y button on the controller.
     * @return The state of the button.
     */
    bool GetYButton() const;
    /**
     * Whether the Y button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetYButtonPressed();
    /**
     * Whether the Y button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetYButtonReleased();


    /**
     * Read the value of the A button on the controller.
     * @return The state of the button.
     */
    bool GetAButton() const;
    /**
     * Whether the A button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetAButtonPressed();
    /**
     * Whether the A button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetAButtonReleased();

    /**
     * Read the value of the B button on the controller.
     * @return The state of the button.
     */
    bool GetBButton() const;
    /**
     * Whether the B button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetBButtonPressed();
    /**
     * Whether the B button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetBButtonReleased();

    /**
     * Read the value of the left trigger 1 button on the controller.
     * @return The state of the button.
     */
    bool GetL1Button() const;
    /**
     * Whether the left trigger 1 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetL1ButtonPressed();
    /**
     * Whether the left trigger 1 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetL1ButtonReleased();

    /**
     * Read the value of the right trigger 1 button on the controller.
     * @return The state of the button.
     */
    bool GetR1Button() const;
    /**
     * Whether the right trigger 1 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetR1ButtonPressed();
    /**
     * Whether the right trigger 1 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetR1ButtonReleased();

    /**
     * Read the value of the left trigger 2 button on the controller.
     * @return The state of the button.
     */
    bool GetL2AsButton() const;
    /**
     * Whether the left trigger 2 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetL2AsButtonPressed();
    /**
     * Whether the left trigger 2 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetL2AsButtonReleased();

    /**
     * Read the value of the right trigger 2 button on the controller.
     * @return The state of the button.
     */
    bool GetR2AsButton() const;
    /**
     * Whether the right trigger 2 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetR2AsButtonPressed();
    /**
     * Whether the right trigger 2 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetR2AsButtonReleased();

    /**
     * Read the value of the left trigger 3 button on the controller.
     * @return The state of the button.
     */
    bool GetL3AsButton() const;
    /**
     * Whether the left trigger 3 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetL3AsButtonPressed();
    /**
     * Whether the left trigger 3 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetL3AsButtonReleased();

    /**
     * Read the value of the right trigger 3 button on the controller.
     * @return The state of the button.
     */
    bool GetR3AsButton() const;
    /**
     * Whether the right trigger 3 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetR3AsButtonPressed();
    /**
     * Whether the right trigger 3 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetR3AsButtonReleased();

    /**
     * Read the value of the options button on the controller.
     * @return The state of the button.
     */
    bool GetOptionsButton() const;
    /**
     * Whether the options button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetOptionsButtonPressed();
    /**
     * Whether the options button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetOptionsButtonReleased();


    /**
     * Read the value of the Start button on the controller.
     * @return The state of the button.
     */
    bool GetStartButton() const;
    /**
     * Whether the Start button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetStartButtonPressed();
    /**
     * Whether the Start button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetStartButtonReleased();

    /**
     * Get the angle in degrees of a POV.
     * The POV angles start at 0 in the up direction, and increase clockwise
     * (e.g. right is 90, upper-left is 315).
     * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
     */
    int GetPOV();

      /**
     * Set the rumble output for the HID.
     * 
     * @param type  Which rumble value to set
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    void SetRumble(RumbleType type, double value);

    /**
     * @brief Checks if the XboxGamepad is connected and matches the expected specifications.
     * 
     * This function verifies the connection status of the XboxGamepad and ensures that
     * it has the expected number of axes, buttons, and POVs. The conditions checked are:
     * - The gamepad is connected.
     * - The gamepad has 6 axes.
     * - The gamepad has 10 buttons.
     * - The gamepad has 1 POV.
     * 
     * @return true if the XboxGamepad is connected and meets the specifications, false otherwise.
     */
    bool IsXboxGamepadConnected();

  /** Represents a digital button on a Xbox gamepad. */
  struct Button {
    /// A button.
    static constexpr int kA = 1;
    /// B button.
    static constexpr int kB = 2;
    /// X button.
    static constexpr int kX = 3;
    /// Y button.
    static constexpr int kY = 4;
    /// Left trigger 1 button.
    static constexpr int kL1 = 5;
    /// Right trigger 1 button.
    static constexpr int kR1 = 6;
    /// Start button.
    static constexpr int kStart = 7;
    /// Options button.
    static constexpr int kOptions = 8;
    /// L3 (left stick) button.
    static constexpr int kL3 = 9;
    /// R3 (right stick) button.
    static constexpr int kR3 = 10;
  };

  /** Represents an axis on a Xbox gamepad. */
  struct Axis {
    /// Left X axis.
    static constexpr int kLeftX = 0;
    /// Left Y axis.
    static constexpr int kLeftY = 1;
    /// Right X axis.
    static constexpr int kRightX = 4;
    /// Right Y axis.
    static constexpr int kRightY = 5;
    /// Left trigger 2.
    static constexpr int kL2 = 2;
    /// Right trigger 2.
    static constexpr int kR2 = 3;
  };

  static constexpr HIDType type = HIDType::kXInputGamepad;
  
//   void InitSendable(wpi::SendableBuilder& builder) override; useless for now
    private :
        double m_thresholdAxisAsButton;

        bool L2AsButtonPressed = false;
        bool R2AsButtonPressed = false;


    protected :
      frc2::Trigger _XButton{[this] { return GetXButton(); }};
      frc2::Trigger _YButton{[this] { return GetYButton(); }};
      frc2::Trigger _AButton{[this] { return GetAButton(); }};
      frc2::Trigger _BButton{[this] { return GetBButton(); }};
      frc2::Trigger _L1Button{[this] { return GetL1Button(); }};
      frc2::Trigger _R1Button{[this] { return GetR1Button(); }};
      frc2::Trigger _L2AsButton{[this] { return GetL2AsButton(); }};
      frc2::Trigger _R2AsButton{[this] { return GetR2AsButton(); }};
      frc2::Trigger _L3AsButton{[this] { return GetL3AsButton(); }};
      frc2::Trigger _R3AsButton{[this] { return GetR3AsButton(); }};
      frc2::Trigger _optionsButton{[this] { return GetOptionsButton(); }};
      frc2::Trigger _startButton{[this] { return GetStartButton(); }};

      frc2::Trigger _upPOVButton{[this] { return GetPOV() == 0; }};
      frc2::Trigger _rightPOVButton{[this] { return GetPOV() == 90; }};
      frc2::Trigger _downPOVButton{[this] { return GetPOV() == 180; }};
      frc2::Trigger _leftPOVButton{[this] { return GetPOV() == 270; }};
      frc2::Trigger _upRightPOVButton{[this] { return GetPOV() == 45; }};
      frc2::Trigger _downRightPOVButton{[this] { return GetPOV() == 135; }};
      frc2::Trigger _downLeftPOVButton{[this] { return GetPOV() == 225; }};
      frc2::Trigger _upLeftPOVButton{[this] { return GetPOV() == 315; }};
};
