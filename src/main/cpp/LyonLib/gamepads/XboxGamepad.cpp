#include "LyonLib/gamepads/XboxGamepad.h" 

XboxGamepad::XboxGamepad(int port)
  : frc::GenericHID(port),
    m_thresholdAxisAsButton(0.5) {
}
XboxGamepad::XboxGamepad(int port, double threshold)
  : frc::GenericHID(port),
    m_thresholdAxisAsButton(threshold) {
}

double XboxGamepad::GetLeftX() const 
{
  return GetRawAxis(Axis::kLeftX);
}
double XboxGamepad::GetLeftY() const 
{
  return GetRawAxis(Axis::kLeftY);
}

double XboxGamepad::GetRightX() const 
{
  return GetRawAxis(Axis::kRightX);
}
double XboxGamepad::GetRightY() const 
{
  return GetRawAxis(Axis::kRightY);
}

double XboxGamepad::GetL2Axis() const 
{
  return GetRawAxis(Axis::kL2);
}
double XboxGamepad::GetR2Axis() const 
{
  return GetRawAxis(Axis::kR2);
}

bool XboxGamepad::GetXButton() const 
{
    return GetRawButton(Button::kX);
}
bool XboxGamepad::GetXButtonPressed() 
{
    return GetRawButtonPressed(Button::kX);
}
bool XboxGamepad::GetXButtonReleased() 
{
    return GetRawButtonReleased(Button::kX);
}

bool XboxGamepad::GetYButton() const 
{
    return GetRawButton(Button::kY);
}
bool XboxGamepad::GetYButtonPressed() 
{
    return GetRawButtonPressed(Button::kY);
}
bool XboxGamepad::GetYButtonReleased() 
{
    return GetRawButtonReleased(Button::kY);
}

bool XboxGamepad::GetAButton() const 
{
    return GetRawButton(Button::kA);
}
bool XboxGamepad::GetAButtonPressed() 
{
    return GetRawButtonPressed(Button::kA);
}
bool XboxGamepad::GetAButtonReleased() 
{
    return GetRawButtonReleased(Button::kA);
}

bool XboxGamepad::GetBButton() const 
{
    return GetRawButton(Button::kB);
}
bool XboxGamepad::GetBButtonPressed() 
{
    return GetRawButtonPressed(Button::kB);
}
bool XboxGamepad::GetBButtonReleased() 
{
    return GetRawButtonReleased(Button::kB);
}

bool XboxGamepad::GetL1Button() const 
{
    return GetRawButton(Button::kL1);
}
bool XboxGamepad::GetL1ButtonPressed() 
{
    return GetRawButtonPressed(Button::kL1);
}
bool XboxGamepad::GetL1ButtonReleased() 
{
    return GetRawButtonReleased(Button::kL1);
}

bool XboxGamepad::GetR1Button() const 
{
    return GetRawButton(Button::kR1);
}
bool XboxGamepad::GetR1ButtonPressed() 
{
    return GetRawButtonPressed(Button::kR1);
}
bool XboxGamepad::GetR1ButtonReleased() 
{
    return GetRawButtonReleased(Button::kR1);
}

bool XboxGamepad::GetL2AsButton() const 
{
    return GetRawAxis(Axis::kL2) > m_thresholdAxisAsButton;
}
bool XboxGamepad::GetL2AsButtonPressed() 
{
    if(GetRawAxis(Axis::kL2) > m_thresholdAxisAsButton)
    {
        if(L2AsButtonPressed)
        {
            return false; // Already pressed
        }
        else
        {
            L2AsButtonPressed = true; // Mark as pressed for next call
            return true;
        }
    }
    else 
    {
        L2AsButtonPressed = false; // Reset pressed state
        return false;
    }
}
bool XboxGamepad::GetL2AsButtonReleased() 
{
    if(GetRawAxis(Axis::kL2) <= m_thresholdAxisAsButton)
    {
        if(!L2AsButtonPressed)
        {
            return false; // Already released
        }
        else
        {
            L2AsButtonPressed = false; // Mark as released for next call
            return true;
        }
    }
    else 
    {
        L2AsButtonPressed = true; // Reset pressed state
        return false;
    }
}

bool XboxGamepad::GetR2AsButton() const 
{
    return GetRawAxis(Axis::kR2) > m_thresholdAxisAsButton;
}
bool XboxGamepad::GetR2AsButtonPressed() 
{
    if(GetRawAxis(Axis::kR2) > m_thresholdAxisAsButton)
    {
        if(R2AsButtonPressed)
        {
            return false; // Already pressed
        }
        else
        {
            R2AsButtonPressed = true; // Mark as pressed for next call
            return true;
        }
    }
    else 
    {
        R2AsButtonPressed = false; // Reset pressed state
        return false;
    }
}
bool XboxGamepad::GetR2AsButtonReleased() 
{
    if(GetRawAxis(Axis::kR2) <= m_thresholdAxisAsButton)
    {
        if(!R2AsButtonPressed)
        {
            return false; // Already released
        }
        else
        {
            R2AsButtonPressed = false; // Mark as released for next call
            return true;
        }
    }
    else 
    {
        R2AsButtonPressed = true; // Reset pressed state
        return false;
    }
}

bool XboxGamepad::GetL3AsButton() const 
{
    return GetRawButton(Button::kL3);
}
bool XboxGamepad::GetL3AsButtonPressed() 
{
    return GetRawButtonPressed(Button::kL3);
}
bool XboxGamepad::GetL3AsButtonReleased() 
{
    return GetRawButtonReleased(Button::kL3);
}

bool XboxGamepad::GetR3AsButton() const 
{
    return GetRawButton(Button::kR3);
}
bool XboxGamepad::GetR3AsButtonPressed() 
{
    return GetRawButtonPressed(Button::kR3);
}
bool XboxGamepad::GetR3AsButtonReleased() 
{
    return GetRawButtonReleased(Button::kR3);
}

bool XboxGamepad::GetOptionsButton() const 
{
    return GetRawButton(Button::kOptions);
}
bool XboxGamepad::GetOptionsButtonPressed() 
{
    return GetRawButtonPressed(Button::kOptions);
}
bool XboxGamepad::GetOptionsButtonReleased() 
{
    return GetRawButtonReleased(Button::kOptions);
}

bool XboxGamepad::GetStartButton() const 
{
    return GetRawButton(Button::kStart);
}
bool XboxGamepad::GetStartButtonPressed() 
{
    return GetRawButtonPressed(Button::kStart);
}
bool XboxGamepad::GetStartButtonReleased() 
{
    return GetRawButtonReleased(Button::kStart);
}

int XboxGamepad::GetPOV() 
{
    return GenericHID::GetPOV(0);
}

void XboxGamepad::SetRumble(RumbleType type, double value) 
{
    GenericHID::SetRumble(type, value);
}

bool XboxGamepad::IsXboxGamepadConnected() 
{
    return  IsConnected() && 
            GetAxisCount() == 6 &&
            GetButtonCount() == 10 &&
            GetPOVCount() == 1;
}