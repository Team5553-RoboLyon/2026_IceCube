#include "subsystems/drivetrain/DrivetrainIOSim.h"
#include "frc/smartdashboard/SmartDashboard.h"

DrivetrainIOSim::DrivetrainIOSim()
{
}

void DrivetrainIOSim::UpdateInputs(DrivetrainIOInputs& inputs)
{
    m_drivetrainSim.Update(0.02_s);
    inputs.isBackLeftMotorConnected = true;
    inputs.isBackRightMotorConnected =true;
    inputs.isFrontLeftMotorConnected =true;
    inputs.isFrontRightMotorConnected = true;

    inputs.backLeftMotorAppliedVoltage  = m_leftVoltage;
    inputs.backRightMotorAppliedVoltage = m_rightVoltage;
    inputs.frontLeftMotorAppliedVoltage = m_leftVoltage;
    inputs.frontRightMotorAppliedVoltage = m_rightVoltage;

    inputs.backLeftMotorBusVoltage = 12_V;
    inputs.backRightMotorBusVoltage =12_V;
    inputs.frontLeftMotorBusVoltage =12_V;
    inputs.frontRightMotorBusVoltage =12_V;

    inputs.backLeftMotorCurrent = m_drivetrainSim.GetLeftCurrentDraw();
    inputs.backRightMotorCurrent =  m_drivetrainSim.GetRightCurrentDraw();
    inputs.frontLeftMotorCurrent =  m_drivetrainSim.GetLeftCurrentDraw();
    inputs.frontRightMotorCurrent = m_drivetrainSim.GetRightCurrentDraw();

    inputs.backLeftMotorTemperature = 23.0_degC;
    inputs.backRightMotorTemperature =  23.0_degC;
    inputs.frontLeftMotorTemperature =  23.0_degC;
    inputs.frontRightMotorTemperature = 23.0_degC;

    inputs.leftSideTraveledDistance = m_drivetrainSim.GetLeftPosition();
    inputs.leftSideVelocity = m_drivetrainSim.GetLeftVelocity();
    inputs.rightSideTraveledDistance = m_drivetrainSim.GetRightPosition();
    inputs.rightSideVelocity = m_drivetrainSim.GetRightVelocity();

    // m_realLeftSideSpeed = inputs.leftSideVelocity.value();
    // m_realRightSideSpeed = inputs.rightSideVelocity.value();
    // inputs.robotPosition = m_odometry.UpdateUsingFusionTwistExp(inputs.leftSideTraveledDistance.value(), inputs.rightSideTraveledDistance.value(), TIME_PER_CYCLE);
    inputs.robotPosition = m_drivetrainSim.GetPose();
    frc::SmartDashboard::PutBoolean("Drivetrain/Motors/BackLeft/isConnected", inputs.isBackLeftMotorConnected);
    frc::SmartDashboard::PutBoolean("Drivetrain/Motors/BackRight/isConnected", inputs.isBackRightMotorConnected);
    frc::SmartDashboard::PutBoolean("Drivetrain/Motors/FrontLeft/isConnected", inputs.isFrontLeftMotorConnected);
    frc::SmartDashboard::PutBoolean("Drivetrain/Motors/FrontRight/isConnected", inputs.isFrontRightMotorConnected);  
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackLeft/AppliedVoltage", inputs.backLeftMotorAppliedVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackRight/AppliedVoltage", inputs.backRightMotorAppliedVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontLeft/AppliedVoltage", inputs.frontLeftMotorAppliedVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontRight/AppliedVoltage", inputs.frontRightMotorAppliedVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackLeft/BusVoltage", inputs.backLeftMotorBusVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackRight/BusVoltage", inputs.backRightMotorBusVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontLeft/BusVoltage", inputs.frontLeftMotorBusVoltage.value()); 
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontRight/BusVoltage", inputs.frontRightMotorBusVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackLeft/Current", inputs.backLeftMotorCurrent.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackRight/Current", inputs.backRightMotorCurrent.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontLeft/Current", inputs.frontLeftMotorCurrent.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontRight/Current", inputs.frontRightMotorCurrent.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackLeft/Temperature", inputs.backLeftMotorTemperature.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/BackRight/Temperature", inputs.backRightMotorTemperature.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontLeft/Temperature", inputs.frontLeftMotorTemperature.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Motors/FrontRight/Temperature", inputs.frontRightMotorTemperature.value());
    frc::SmartDashboard::PutNumber("Drivetrain/LeftSide/TraveledDistance", inputs.leftSideTraveledDistance.value());
    frc::SmartDashboard::PutNumber("Drivetrain/LeftSide/Velocity", inputs.leftSideVelocity.value());
    frc::SmartDashboard::PutNumber("Drivetrain/RightSide/TraveledDistance", inputs.rightSideTraveledDistance.value());
    frc::SmartDashboard::PutNumber("Drivetrain/RightSide/Velocity", inputs.rightSideVelocity.value());
    robotPoseLogger.Log(inputs.robotPosition);
}

void DrivetrainIOSim::SetVoltage(const units::volt_t leftSideVoltage, const units::volt_t rightSideVoltage)
{
    DEBUG_ASSERT((leftSideVoltage.value() <= driveConstants::Motors::VOLTAGE_COMPENSATION) 
        && (leftSideVoltage.value() >= -driveConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Drivetrain left side Voltage out of range");

    DEBUG_ASSERT((rightSideVoltage.value() <= driveConstants::Motors::VOLTAGE_COMPENSATION) 
        && (rightSideVoltage.value() >= -driveConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Drivetrain right side Voltage out of range");
    
    m_drivetrainSim.SetInputs(leftSideVoltage, rightSideVoltage);
    m_leftVoltage = leftSideVoltage;
    m_rightVoltage = rightSideVoltage;
}

void DrivetrainIOSim::SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle)
{
    DEBUG_ASSERT((leftSideDutyCycle <= 1.0) && (leftSideDutyCycle >= -1.0) 
            ,"Drivetrain left side Duty Cycle out of range");

    DEBUG_ASSERT((rightSideDutyCycle <= 1.0) && (rightSideDutyCycle >= -1.0) 
            ,"Drivetrain right side Duty Cycle out of range");
    
    m_drivetrainSim.SetInputs(units::volt_t(leftSideDutyCycle * driveConstants::Motors::VOLTAGE_COMPENSATION), 
                            units::volt_t(rightSideDutyCycle * driveConstants::Motors::VOLTAGE_COMPENSATION));
    m_leftVoltage = units::volt_t(leftSideDutyCycle * driveConstants::Motors::VOLTAGE_COMPENSATION);
    m_rightVoltage = units::volt_t(rightSideDutyCycle * driveConstants::Motors::VOLTAGE_COMPENSATION);       
}

void DrivetrainIOSim::SetChassisSpeed(const frc::ChassisSpeeds &speeds)
{
    DEBUG_ASSERT(speeds.vy() == 0.0, "Are You stupid ? Did you know that a tank can't move on the Y axis ?");

    //Differential :
    // Vr = Vf + Rb * omega
    // Vl = Vf - Rb * omega
    double rightSideSpeed = speeds.vx() + driveConstants::Specifications::BASE_TRACK_RADIUS * speeds.omega();
    double leftSideSpeed = speeds.vx() - driveConstants::Specifications::BASE_TRACK_RADIUS * speeds.omega();
 
    double highestSpeedSide = NMAX(NABS(rightSideSpeed), NABS(leftSideSpeed));
    if(highestSpeedSide > driveConstants::Specifications::MAX_LINEAR_SPEED)
    {
        double scaleFactor = driveConstants::Specifications::MAX_LINEAR_SPEED / highestSpeedSide;
        rightSideSpeed *= scaleFactor;
        leftSideSpeed *= scaleFactor;
    }
    
    double rightOutput = rightSideSpeed * driveConstants::Specifications::LINEAR_TO_MOTOR_SPEED_FACTOR;
    double leftOutput = leftSideSpeed * driveConstants::Specifications::LINEAR_TO_MOTOR_SPEED_FACTOR;

    // HACK : speed to voltage
    m_leftVoltage = units::volt_t(leftOutput / driveConstants::Specifications::MOTOR_FREE_SPEED * driveConstants::Motors::VOLTAGE_COMPENSATION);
    m_rightVoltage = units::volt_t(rightOutput / driveConstants::Specifications::MOTOR_FREE_SPEED * driveConstants::Motors::VOLTAGE_COMPENSATION);
    m_drivetrainSim.SetInputs(m_leftVoltage, m_rightVoltage);
}

void DrivetrainIOSim::ResetPosition(const frc::Pose2d position)
{
    m_drivetrainSim.SetPose(position);
    // m_odometry.ResetPose2D(position);
}