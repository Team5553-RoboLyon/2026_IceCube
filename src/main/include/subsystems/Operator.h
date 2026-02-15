#include "LyonLib/gamepads/RevGamepad.h"

class Operator final : public RevGamepad
{

#if (OPERATOR == (ADAM))
private:

public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger STOWED{_crossButton };
    frc2::Trigger ARMED_TO_CLIMB{_optionsButton };
    frc2::Trigger CLIMBED{_circleButton };
    frc2::Trigger toggle{_triangleButton };
#elif (OPERATOR == (VICTOR))
private:

public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

#endif


    void SetRumble(RumbleType type, double value)
    {
        RevGamepad::SetRumble(type, value);
    }
    void SetRumble(double value)
    {
        RevGamepad::SetRumble(RumbleType::kBothRumble, value);
    }
    void SetRumble(RumbleType type)
    {
        RevGamepad::SetRumble(type, 1.0);
    }

};