#include "LyonLib/gamepads/XboxGamepad.h"

class Operator final : public XboxGamepad
{

#if (OPERATOR == (ADAM))
private:

public:

    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
    ~Operator() override = default;

#elif (OPERATOR == (VICTOR))
private:

public:

    frc2::Trigger STAND_BY{_YButton};
    frc2::Trigger PROTECT_INTAKE{_startButton};
    frc2::Trigger STOP_SHOOT{_R2AsButton};
    frc2::Trigger STOP_INTAKE{_L2AsButton};
    frc2::Trigger REFUEL{_L2AsButton};
    frc2::Trigger SHOOT_TO_HUB{_R2AsButton};
    frc2::Trigger PREPARE_CLIMB{_R1Button};
    frc2::Trigger RETRACT_CLIMB{_L1Button};
    frc2::Trigger CLIMB{_L3AsButton};
    frc2::Trigger PREPARE_REFUEL{_XButton};
    frc2::Trigger RETRACT_INTAKE{_BButton};
    frc2::Trigger TOGGLE_IRBREAKER_VALUE{_R3AsButton};
    
    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
    ~Operator() override = default;

#endif


    void SetRumble(RumbleType type, double value)
    {
        XboxGamepad::SetRumble(type, value);
    }
    void SetRumble(double value)
    {
        XboxGamepad::SetRumble(RumbleType::kBothRumble, value);
    }
    void SetRumble(RumbleType type)
    {
        XboxGamepad::SetRumble(type, 1.0);
    }

};