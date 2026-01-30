#pragma once

struct ShooterIOInputs
{
    bool isLeftMotorConnected = true;
    
    double LeftMotorAppliedVoltage = 0.0;
    double LeftMotorBusVoltage = 0.0;
    double LeftMotorCurrent = 0.0;
    double LeftMotorTemperature = 0.0;
    double LeftMotorEncoderVelocity = 0.0;

    bool isRightMotorConnected = true;
    
    double RightMotorAppliedVoltage = 0.0;
    double RightMotorBusVoltage = 0.0;
    double RightMotorCurrent = 0.0;
    double RightMotorTemperature = 0.0;
    double RightMotorEncoderVelocity = 0.0;

    bool isBottomMotorConnected = true;
    
    double BottomMotorAppliedVoltage = 0.0;
    double BottomMotorBusVoltage = 0.0;
    double BottomMotorCurrent = 0.0;
    double BottomMotorTemperature = 0.0;
    double BottomMotorEncoderVelocity = 0.0;

    double rotation = 0.0;
    double ShooterVelocity = 0.0;
    bool FuelLaunched = false;
};


class ShooterIO {
public:
    virtual ~ShooterIO() = default;

    virtual void UpdateInputs(ShooterIOInputs& inputs) = 0;

    virtual void SetVoltage(double upVoltage, double bottomVoltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;

    virtual void ResetRotation() = 0;

};