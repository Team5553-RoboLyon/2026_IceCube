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
    frc2::Trigger SHOOT_TO_HUB{_R2AsButton};
    frc2::Trigger FEED_ALLY{_XButton};
    frc2::Trigger STOP{_BButton};
    frc2::Trigger REVERSE{_L2AsButton};
    frc2::Trigger KEEP_ALL_FOR_YOU{_AButton};
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