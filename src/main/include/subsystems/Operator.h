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
    frc2::Trigger FOLLOW_HUB{_AButton};
    frc2::Trigger POINT_AT_ALLIANCE_ZONE{_BButton};
    frc2::Trigger PREPARE_EJECT{_XButton};

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