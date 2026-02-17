#include "LyonLib/gamepads/XboxGamepad.h"

class Operator final : public XboxGamepad
{

#if (OPERATOR == (ADAM))
private:

public:
    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger STAND_BY{_XButton};
    frc2::Trigger PREPARE_SHOOT{_AButton};
    frc2::Trigger EVACUATE_SHOOTER{_BButton};
    frc2::Trigger FEED_SHOOTER{_YButton};
    frc2::Trigger SET_IRBREAKER_TRUE{_R1Button};
    frc2::Trigger SET_IRBREAKER_FALSE{_L1Button};

#elif (OPERATOR == (VICTOR))
private:

public:
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