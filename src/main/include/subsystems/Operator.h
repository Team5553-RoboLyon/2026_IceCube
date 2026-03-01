#include "Constants.h"

#if (OPERATOR == (ADAM))
#include "LyonLib/gamepads/RevGamepad.h"
class Operator final : public RevGamepad
{
private:


public:

    frc2::Trigger STAND_BY{_triangleButton};
    frc2::Trigger REFUEL{_circleButton};
    frc2::Trigger EJECT{_crossButton};
    frc2::Trigger BECOME_AN_INDEXER{_squareButton};
    frc2::Trigger EXTEND{_L1Button};
    frc2::Trigger RETURN_AT_HOME{_R1Button};
    frc2::Trigger PROTECT_YOURSELF{_optionsButton};
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger STOWED{_crossButton };
    frc2::Trigger ARMED_TO_CLIMB{_optionsButton };
    frc2::Trigger CLIMBED{_circleButton };
    frc2::Trigger toggle{_triangleButton };

    

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
    frc2::Trigger REFUEL{_R2AsButton};
    frc2::Trigger EJECT{_L2AsButton};
    frc2::Trigger BECOME_AN_INDEXER{_XButton};
    frc2::Trigger EXTEND{_L1Button};
    frc2::Trigger RETURN_AT_HOME{_R1Button};
    frc2::Trigger PROTECT_YOURSELF{_optionsButton};

    frc2::Trigger STOWED{_R3AsButton };
    frc2::Trigger ARMED_TO_CLIMB{_L3AsButton};
    frc2::Trigger CLIMBED{_startButton};
    // frc2::Trigger toggle{_triangleButton };
    // frc2::Trigger STAND_BY{_YButton};
    // frc2::Trigger SHOOT_TO_HUB{_R2AsButton};
    // frc2::Trigger FEED_ALLY{_XButton};
    // frc2::Trigger STOP{_BButton};
    // frc2::Trigger REVERSE{_L2AsButton};
    // frc2::Trigger KEEP_ALL_FOR_YOU{_AButton};
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