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

    frc2::Trigger PREPARE_REFUEL{_R1Button};
    frc2::Trigger RETRACT_INTAKE{_L1Button};
    frc2::Trigger CLIMB{_R3AsButton};
    frc2::Trigger RETRACT_CLIMBER{_L3AsButton};
    frc2::Trigger PREPARE_CLIMB{_startButton};
    frc2::Trigger SHOOT_TO_HUB{_R2AsButton};
    frc2::Trigger SHOOT_AND_REFUEL{_AButton};
    frc2::Trigger STOP_SHOOT{_BButton};
    frc2::Trigger EVACUATE_SHOOTER{_YButton};
    frc2::Trigger PROTECT_INTAKE{_optionsButton};
    
    Operator(int port) : XboxGamepad(port){};
    Operator(int port, double threshold) : XboxGamepad(port, threshold){};
    ~Operator() override = default;

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
#endif
};