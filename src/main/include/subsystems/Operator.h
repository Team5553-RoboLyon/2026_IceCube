#include "LyonLib/gamepads/RevGamepad.h"

class Operator final : public RevGamepad
{

#if (OPERATOR == (ADAM))
private:

public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

#elif (OPERATOR == (VICTOR))
private:

public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger stopShooter{_crossButton};
    frc2::Trigger startShooter{_squareButton};

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