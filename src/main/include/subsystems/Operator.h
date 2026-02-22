#include "Constants.h"


#if (OPERATOR == (ADAM))
#include "LyonLib/gamepads/RevGamepad.h"
class Operator final : public RevGamepad
{
private:


public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger STAND_BY{_squareButton};
    frc2::Trigger SHOOT_TO_HUB{_triangleButton};
    frc2::Trigger FEED_ALLY{_circleButton};
    frc2::Trigger STOP{_triangleButton};
    frc2::Trigger REVERSE{_shareButton};
    frc2::Trigger KEEP_ALL_FOR_YOU{_optionsButton};

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

#elif (OPERATOR == (VICTOR))
#include "LyonLib/gamepads/XboxGamepad.h"
class Operator final : public XboxGamepad
{
private:


public:
    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger STAND_BY{_YButton};
    frc2::Trigger SHOOT_TO_HUB{_R3AsButton};
    frc2::Trigger FEED_ALLY{_XButton};
    frc2::Trigger STOP{_BButton};
    frc2::Trigger REVERSE{_L3AsButton};
    frc2::Trigger KEEP_ALL_FOR_YOU{_AButton};


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
#endif