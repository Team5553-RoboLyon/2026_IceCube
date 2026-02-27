#include "subsystems/drivetrain/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "LyonLib/utils/MacroUtilsRBL.h"


DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainIO *pIO) 
: DrivetrainSubsystem(pIO,
                    []() { return 0.0; },
                    []() { return 0.0; },
                    []() { return false; },
                    []() { return false; })
{
    m_axesAreActive = false;
}

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainIO *pIO, 
                    std::function<double()> fxForwardAxis,
                    std::function<double()> fxRotationAxis,
                    std::function<bool()> fxSlowDriveButton,
                    std::function<bool()> fxDriveActionButton)
                    : m_pTankDriveIO(pIO), 
                    m_fxForwardAxis(fxForwardAxis),
                    m_fxRotationAxis(fxRotationAxis),
                    m_fxSlowDriveButton(fxSlowDriveButton),
                    m_fxDriveActionButton(fxDriveActionButton),
                    m_axesAreActive(true)
{
    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        [this](){ return GetOdometryPose(); }, //TODO : replace with robot pose
        [this](const frc::Pose2d &pose){ m_pTankDriveIO->ResetPosition(pose); },
        [this](){ return m_pTankDriveIO->GetChassisSpeed(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](const frc::ChassisSpeeds &speeds){ m_pTankDriveIO->SetChassisSpeed(speeds); },
        std::make_shared<pathplanner::PPLTVController>(Q,R,0.02_s), 
        config, // The robot configuration
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}
void DrivetrainSubsystem::SetWantedDrive(const DriveMode wantedDrive)
{
    m_wantedDrive = wantedDrive;
}

DrivetrainSubsystem::SystemDrive DrivetrainSubsystem::GetSystemDrive() const
{
    return m_systemDrive;
}

void DrivetrainSubsystem::ConfigureManualControlInputsAxis(const std::function<double()> fxForwardAxis,
                                            const std::function<double()> fxRotationAxis,
                                            const std::function<bool()> fxSlowDriveButton,
                                            const std::function<bool()> fxDriveActionButton)
{
    m_fxForwardAxis = fxForwardAxis;
    m_fxRotationAxis = fxRotationAxis;
    m_fxSlowDriveButton = fxSlowDriveButton;
    m_fxDriveActionButton = fxDriveActionButton;
    m_axesAreActive = true;
}

void DrivetrainSubsystem::ResetOdometryPose(const frc::Pose2d pose)
{
    m_pTankDriveIO->ResetPosition(pose);
}

void DrivetrainSubsystem::Periodic()
{
    m_pTankDriveIO->UpdateInputs(inputs);
    
    m_frontLeftMotorDisconnected.Set(!inputs.isFrontLeftMotorConnected);
    m_frontRightMotorDisconnected.Set(!inputs.isFrontRightMotorConnected);
    m_backLeftMotorDisconnected.Set(!inputs.isBackLeftMotorConnected);
    m_backRightMotorDisconnected.Set(!inputs.isBackRightMotorConnected);

    m_frontLeftMotorHot.Set(inputs.frontLeftMotorTemperature.value() > driveConstants::Motors::HOT_THRESHOLD);
    m_frontRightMotorHot.Set(inputs.frontRightMotorTemperature.value() > driveConstants::Motors::HOT_THRESHOLD);
    m_backLeftMotorHot.Set(inputs.backLeftMotorTemperature.value() > driveConstants::Motors::HOT_THRESHOLD);
    m_backRightMotorHot.Set(inputs.backRightMotorTemperature.value() > driveConstants::Motors::HOT_THRESHOLD);

    m_frontLeftMotorOverheating.Set(inputs.frontLeftMotorTemperature.value() > driveConstants::Motors::OVERHEATING_THRESHOLD);
    m_frontRightMotorOverheating.Set(inputs.frontRightMotorTemperature.value() > driveConstants::Motors::OVERHEATING_THRESHOLD);
    m_backLeftMotorOverheating.Set(inputs.backLeftMotorTemperature.value() > driveConstants::Motors::OVERHEATING_THRESHOLD);
    m_backRightMotorOverheating.Set(inputs.backRightMotorTemperature.value() > driveConstants::Motors::OVERHEATING_THRESHOLD);


    switch (m_wantedDrive) //Handle State transition
    {
    case DriveMode::ARCADE_DRIVE :
        if(m_systemDrive != SystemDrive::ARCADE_DRIVE)
        {
            m_rotationSigma = 0.0;
            m_forwardLimitedAxis.Reset();
            m_rotationLimitedAxis.Reset();
            m_forwardLimitedAxis.SetRateLimit(driveConstants::ArcadeDrive::TIME_TO_REACH_FULL_FORWARD,
                                            driveConstants::ArcadeDrive::TIME_TO_STOP_FORWARD);
            m_rotationLimitedAxis.SetRateLimit(driveConstants::ArcadeDrive::TIME_TO_REACH_FULL_ROTATION,
                                            driveConstants::ArcadeDrive::TIME_TO_STOP_ROTATION);
            m_systemDrive = SystemDrive::ARCADE_DRIVE;
        }
        break;
    
    case DriveMode::CURVE_DRIVE :
        if(m_systemDrive != SystemDrive::CURVE_DRIVE)
        {
            m_forwardLimitedAxis.Reset();
            m_forwardLimitedAxis.SetRateLimit(driveConstants::CurveDrive::TIME_TO_REACH_FULL_FORWARD,
                                            driveConstants::CurveDrive::TIME_TO_STOP_FORWARD);
            m_previousRotation = 0.0;
            m_negInertiaAccumulator = 0.0;
            m_quickStopAccumulator = 0.0;
            m_systemDrive = SystemDrive::CURVE_DRIVE;
        }
        break;

    case DriveMode::AUTO_PATH_FOLLOWER :
        if(m_systemDrive != SystemDrive::AUTO_PATH_FOLLOWER)
        {
            m_systemDrive = SystemDrive::AUTO_PATH_FOLLOWER;
            m_autoTimer.Restart();
        }
        break;

    case DriveMode::DISABLE :
        if(m_systemDrive != SystemDrive::DISABLE)
        {
            m_systemDrive = SystemDrive::DISABLE;
        }
        break;

    default:
        DEBUG_ASSERT(false, "DrivetrainSubsystem::Periodic: Invalid WantedDrive state");
        break;
    }

    switch (m_systemDrive) //Calculate output from SystemDrive
    {
    case SystemDrive::AUTO_PATH_FOLLOWER:
        // m_output = FollowPath();
        break;

    case SystemDrive::ARCADE_DRIVE :
        m_output = ArcadeDrive(GetPercentages());
        m_pTankDriveIO->SetChassisSpeed(m_output);
        break;
    
    case SystemDrive::CURVE_DRIVE :
        m_output = CurveDrive(GetPercentages(), m_fxDriveActionButton());
        m_pTankDriveIO->SetChassisSpeed(m_output);
        break;
    
    case SystemDrive::DISABLE :
        m_output = restSpeeds;
        m_pTankDriveIO->SetChassisSpeed(m_output);
        break;
    
    default:
        DEBUG_ASSERT(false, "tu n'es pas censé lire ça. Sinon bravo, tu as activé une assert !");
        break;
    }

    frc::SmartDashboard::PutNumber("Drivetrain/SystemDrive", (int)m_systemDrive);
    frc::SmartDashboard::PutNumber("Drivetrain/WantedDrive", (int)m_wantedDrive);
    frc::SmartDashboard::PutNumber("Drivetrain/Timestamp", m_autoTimer.GetElapsedTimeSeconds()); 
}

frc::ChassisSpeeds DrivetrainSubsystem::ArcadeDrive(const std::pair<double, double> percentage)
{ 
    frc::ChassisSpeeds output;

    if(m_fxDriveActionButton())
    {
        m_forwardLimitedAxis.Update(-percentage.first);
    }
    else
    {
        m_forwardLimitedAxis.Update(percentage.first);
    }
    m_rotationLimitedAxis.Update(percentage.second);

    m_rotationSigma = NLERP(driveConstants::ArcadeDrive::MIN_ROTATION_SIGMA,
                            driveConstants::ArcadeDrive::MAX_ROTATION_SIGMA, 
                            NABS(m_rotationLimitedAxis.GetCurrentSpeed()));

    output.vx = (units::velocity::meters_per_second_t)
                std::sin(m_forwardLimitedAxis.GetCurrentSpeed() * (NF64_PI_2)) * 
                driveConstants::Specifications::MAX_LINEAR_SPEED;

    output.vy = (units::velocity::meters_per_second_t)0.0; // Ce n'est pas des swerves donc pas de Vy pour cette fois, dsl

    output.omega = (units::angular_velocity::radians_per_second_t)
                    std::sin(m_rotationLimitedAxis.GetCurrentSpeed() * (NF64_PI_2)) * m_rotationSigma *
                    driveConstants::Specifications::MAX_ROTATION_SPEED;
    
    return output;
}

frc::ChassisSpeeds DrivetrainSubsystem::CurveDrive(const std::pair<double, double> percentage, const bool quickTurnEnabled)
{
    frc::ChassisSpeeds output;
    bool QuickTurn = quickTurnEnabled;

    // Deadband → activate quickTurn
    if(NABS(percentage.first) < driveConstants::Settings::DEADBAND)
    {
        QuickTurn = true;
    }

    m_forwardLimitedAxis.Update(percentage.first);

    //R1 = sin(PI/2 * SIN_CURVE_I * R) / sin(PI/2 * SIN_CURVE_I)
    //Rnlr = sin(PI/2 * SIN_CURVE_I * R1) / sin(PI/2 * SIN_CURVE_I)
    double nonLinearRotation = std::sin((std::sin((NF64_PI_2) * driveConstants::CurveDrive::SINUSOIDAL_CURVATURE_INTENSITY * percentage.second)
                                / driveConstants::CurveDrive::DENOMINATOR) * (NF64_PI_2) * driveConstants::CurveDrive::SINUSOIDAL_CURVATURE_INTENSITY) 
                                / driveConstants::CurveDrive::DENOMINATOR;

    
    if(QuickTurn)
    {
        output.omega = (units::angular_velocity::radians_per_second_t)nonLinearRotation * NABS(nonLinearRotation) *
                        driveConstants::Specifications::MAX_ROTATION_SPEED;
        m_quickStopAccumulator = (1 - driveConstants::CurveDrive::QUICK_STOP_ALPHA) * m_quickStopAccumulator +
                                driveConstants::CurveDrive::QUICK_STOP_ALPHA * 
                                std::clamp(nonLinearRotation * NABS(nonLinearRotation), -1.0, 1.0) * 2.0;
    }
    else
    {
        double negInertia = (percentage.second - m_previousRotation) * driveConstants::CurveDrive::NEG_INERTIA_SCALAR;
        m_negInertiaAccumulator += negInertia;

        double angularPower = std::abs(percentage.first) *
                              (nonLinearRotation + m_negInertiaAccumulator) *
                               driveConstants::CurveDrive::TURN_SENSITIVITY -
                              m_quickStopAccumulator;

        if (m_negInertiaAccumulator > 1.0)       m_negInertiaAccumulator -= 1.0;
        else if (m_negInertiaAccumulator < -1.0) m_negInertiaAccumulator += 1.0;
        else                                     m_negInertiaAccumulator = 0.0;

        if (m_quickStopAccumulator > 1.0)       m_quickStopAccumulator -= 1.0;
        else if (m_quickStopAccumulator < -1.0) m_quickStopAccumulator += 1.0;
        else                                    m_quickStopAccumulator = 0.0;
        
        output.vx = (units::velocity::meters_per_second_t)m_forwardLimitedAxis.GetCurrentSpeed() *
                    driveConstants::Specifications::MAX_LINEAR_SPEED;
        
        output.vy = (units::velocity::meters_per_second_t)0.0; // Ce n'est pas des swerves donc pas de Vy pour cette fois, dsl

        output.omega = (units::angular_velocity::radians_per_second_t)angularPower * 
                        driveConstants::Specifications::MAX_ROTATION_SPEED;
    }

    m_previousRotation = percentage.second;
    return output;
}

frc::ChassisSpeeds DrivetrainSubsystem::FollowPath()
{
    frc::ChassisSpeeds output{};
    // // Verify if there is a sample to follow
    // if (!m_autoSampleToBeApplied.has_value()) {
    //     return output; // Nothing to do
    // }

    // // Current target sample from the trajectory
    // const choreo::DifferentialSample& currentSample = m_autoSampleToBeApplied.value();
    // const frc::ChassisSpeeds targetSpeeds = currentSample.GetChassisSpeeds();
    // const frc::Pose2d targetPose = currentSample.GetPose();
    // m_ErreurLogger.Log(inputs.robotPosition.RelativeTo(targetPose));
    // m_theoriticalLogger.Log(targetPose);
     
    // output = m_ltvController.Calculate(inputs.robotPosition, targetPose, targetSpeeds.vx, targetSpeeds.omega);
    return output;
}

std::pair<double, double> DrivetrainSubsystem::GetPercentages()
{
    std::pair<double, double> output;
    double fwdPercentage = NCLAMP(-1.0, m_fxForwardAxis(), 1.0);
    double rotationPercentage = NCLAMP(-1.0, m_fxRotationAxis(), 1.0);

    if (m_fxSlowDriveButton()) 
    {
        fwdPercentage /= driveConstants::Settings::SLOW_RATE;
        rotationPercentage /= driveConstants::Settings::SLOW_RATE;
    }

    output.first = fwdPercentage;
    output.second = rotationPercentage;
    return output;
}