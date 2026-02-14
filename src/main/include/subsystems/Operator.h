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
    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger stopShooter{_crossButton};
    frc2::Trigger startShooter{_squareButton};

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