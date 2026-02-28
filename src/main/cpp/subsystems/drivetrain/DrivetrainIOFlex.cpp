#include "subsystems/drivetrain/DrivetrainIOFlex.h"
#include "frc/smartdashboard/SmartDashboard.h"

DrivetrainIOFlex::DrivetrainIOFlex()
{
    // Set the back left motor configs
    m_motorBackLeftConfig.SetIdleMode(driveConstants::Motors::IDLE_MODE)
        .Inverted(driveConstants::Motors::LEFT_MOTOR_INVERTED)
        .SmartCurrentLimit(driveConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::Motors::RAMP_RATE)
        .VoltageCompensation(driveConstants::Motors::VOLTAGE_COMPENSATION);

    // Set the back right motor configs
    m_motorBackRightConfig.SetIdleMode(driveConstants::Motors::IDLE_MODE)
        .Inverted(driveConstants::Motors::RIGHT_MOTORS_INVERTED)
        .SmartCurrentLimit(driveConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::Motors::RAMP_RATE)
        .VoltageCompensation(driveConstants::Motors::VOLTAGE_COMPENSATION);

    m_motorFrontLeftConfig.Apply(m_motorBackLeftConfig).Follow(m_motorBackLeft);
    m_motorFrontRightConfig.Apply(m_motorBackRightConfig).Follow(m_motorBackRight);
    
    m_motorBackLeft.Configure(m_motorBackLeftConfig, 
                            rev::ResetMode::kResetSafeParameters, 
                            rev::PersistMode::kNoPersistParameters);
    m_motorBackRight.Configure(m_motorBackRightConfig, 
                            rev::ResetMode::kResetSafeParameters, 
                            rev::PersistMode::kNoPersistParameters);
    m_motorFrontLeft.Configure(m_motorFrontLeftConfig, 
                            rev::ResetMode::kResetSafeParameters, 
                            rev::PersistMode::kNoPersistParameters);
    m_motorFrontRight.Configure(m_motorFrontRightConfig, 
                            rev::ResetMode::kResetSafeParameters, 
                            rev::PersistMode::kNoPersistParameters);

    m_motorBackLeft.ClearFaults();
    m_motorBackRight.ClearFaults();
    m_motorFrontLeft.ClearFaults(); 
    m_motorFrontRight.ClearFaults();

    m_encoderLeft.SetDistancePerPulse(driveConstants::Encoder::DISTANCE_PER_PULSE);
    m_encoderRight.SetDistancePerPulse(driveConstants::Encoder::DISTANCE_PER_PULSE);
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
}

void DrivetrainIOFlex::UpdateInputs(DrivetrainIOInputs& inputs)
{
    inputs.isBackLeftMotorConnected = (m_motorBackLeft.GetBusVoltage() !=0.0) && !m_motorBackLeft.GetFaults().can;
    inputs.isBackRightMotorConnected = (m_motorBackRight.GetBusVoltage() != 0.0) && !m_motorBackRight.GetFaults().can;
    inputs.isFrontLeftMotorConnected = (m_motorFrontLeft.GetBusVoltage() != 0.0) && !m_motorFrontLeft.GetFaults().can;
    inputs.isFrontRightMotorConnected = (m_motorFrontRight.GetBusVoltage() != 0.0) && !m_motorFrontRight.GetFaults().can;

    inputs.backLeftMotorAppliedVoltage = units::volt_t(m_motorBackLeft.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION);
    inputs.backRightMotorAppliedVoltage = units::volt_t(m_motorBackRight.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION);
    inputs.frontLeftMotorAppliedVoltage = units::volt_t(m_motorFrontLeft.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION);
    inputs.frontRightMotorAppliedVoltage = units::volt_t(m_motorFrontRight.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION);

    inputs.backLeftMotorBusVoltage = units::volt_t(m_motorBackLeft.GetBusVoltage());
    inputs.backRightMotorBusVoltage = units::volt_t(m_motorBackRight.GetBusVoltage());
    inputs.frontLeftMotorBusVoltage = units::volt_t(m_motorFrontLeft.GetBusVoltage());
    inputs.frontRightMotorBusVoltage = units::volt_t(m_motorFrontRight.GetBusVoltage());

    inputs.backLeftMotorCurrent = units::ampere_t(m_motorBackLeft.GetOutputCurrent());
    inputs.backRightMotorCurrent = units::ampere_t(m_motorBackRight.GetOutputCurrent());
    inputs.frontLeftMotorCurrent = units::ampere_t(m_motorFrontLeft.GetOutputCurrent());
    inputs.frontRightMotorCurrent = units::ampere_t(m_motorFrontRight.GetOutputCurrent());

    inputs.backLeftMotorTemperature = units::celsius_t(m_motorBackLeft.GetMotorTemperature());
    inputs.backRightMotorTemperature = units::celsius_t(m_motorBackRight.GetMotorTemperature());
    inputs.frontLeftMotorTemperature = units::celsius_t(m_motorFrontLeft.GetMotorTemperature());
    inputs.frontRightMotorTemperature = units::celsius_t(m_motorFrontRight.GetMotorTemperature());

    inputs.leftSideTraveledDistance = units::meter_t(m_encoderLeft.GetDistance());
    inputs.leftSideVelocity = units::meters_per_second_t(m_encoderLeft.GetRate());
    inputs.rightSideTraveledDistance = units::meter_t(m_encoderRight.GetDistance());
    inputs.rightSideVelocity = units::meters_per_second_t(m_encoderRight.GetRate());

    m_realLeftSideSpeed = inputs.leftSideVelocity.value();
    m_realRightSideSpeed = inputs.rightSideVelocity.value();

    inputs.odometryPosition = m_odometry.UpdateUsingFusionTwistExp(inputs.leftSideTraveledDistance.value(), inputs.rightSideTraveledDistance.value(), TIME_PER_CYCLE);

    #ifdef DRIVETRAIN_SMARTDASHBOARD_LOG
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
    #else
    m_logger.Log(inputs);
    #endif

    m_odometryPoseLogger.Log(inputs.odometryPosition);

}

void DrivetrainIOFlex::SetVoltage(const units::volt_t leftSideVoltage, const units::volt_t rightSideVoltage)
{
    DEBUG_ASSERT((leftSideVoltage.value() <= driveConstants::Motors::VOLTAGE_COMPENSATION) 
        && (leftSideVoltage.value() >= -driveConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Drivetrain left side Voltage out of range");

    DEBUG_ASSERT((rightSideVoltage.value() <= driveConstants::Motors::VOLTAGE_COMPENSATION) 
        && (rightSideVoltage.value() >= -driveConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Drivetrain right side Voltage out of range");
    
    m_motorBackLeft.SetVoltage(leftSideVoltage);
    m_motorBackRight.SetVoltage(rightSideVoltage);
}

void DrivetrainIOFlex::SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle)
{
    DEBUG_ASSERT((leftSideDutyCycle <= 1.0) && (leftSideDutyCycle >= -1.0) 
            ,"Drivetrain left side Duty Cycle out of range");

    DEBUG_ASSERT((rightSideDutyCycle <= 1.0) && (rightSideDutyCycle >= -1.0) 
            ,"Drivetrain right side Duty Cycle out of range");
    
    m_motorBackLeft.Set(leftSideDutyCycle);
    m_motorBackRight.Set(rightSideDutyCycle);
}

void DrivetrainIOFlex::SetChassisSpeed(const frc::ChassisSpeeds &speeds)
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

    //HACK : speed to voltage
    m_motorBackLeft.SetVoltage(units::volt_t(leftOutput / driveConstants::Specifications::MOTOR_FREE_SPEED 
                                            * driveConstants::Motors::VOLTAGE_COMPENSATION));
    m_motorBackRight.SetVoltage(units::volt_t(rightOutput / driveConstants::Specifications::MOTOR_FREE_SPEED 
                                            * driveConstants::Motors::VOLTAGE_COMPENSATION));
}

void DrivetrainIOFlex::ResetPosition(const frc::Pose2d& position)
{
    m_encoderLeft.Reset();
    m_encoderRight.Reset();

    m_odometry.ResetPose2D(position);
}