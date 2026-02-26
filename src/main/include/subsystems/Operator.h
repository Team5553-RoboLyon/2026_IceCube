#include "Constants.h"


#if (OPERATOR == (ADAM))
#include "LyonLib/gamepads/RevGamepad.h"
class Operator final : public RevGamepad
{
private:


public:

    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
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
        XboxGamepad::SetRumble(type, value);
    }
    void SetRumble(double value)
    {
        XboxGamepad::SetRumble(RumbleType::kBothRumble, value);
        XboxGamepad::SetRumble(RumbleType::kBothRumble, value);
    }
    void SetRumble(RumbleType type)
    {
        XboxGamepad::SetRumble(type, 1.0);
        XboxGamepad::SetRumble(type, 1.0);
    }

};
#endif