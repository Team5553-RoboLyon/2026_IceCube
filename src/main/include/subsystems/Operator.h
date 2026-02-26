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