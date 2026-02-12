#pragma once

#include "PhotonCamera.h"

/**
 * This class is a chlidren class of the PhotonCamera adapted to
 * an AprilTag detection pipiline. It allows you to get some more
 * data related to April Tags.
 */
class AprilTagPhotonCamera : public PhotonCamera
{
    public :
    
    using PhotonCamera::PhotonCamera;

    /**
     * Get the ID of the currently analysed April Tag target
     * 
     * @return the ID of the currently analysed April Tag target
     */
    int GetTargetID();

    /**
     * Get the pose ambiguity of the currently April Tag target
     * 
     * @return the pose ambiguity of the currently April Tag target
     */
    double GetTargetPoseAmbiguity();

    /**
     * Override the PhotonCamera::GetTargetSkew function because it can't be used
     * with an April Tag function
     * 
     * @return 0 as an error output and send a message to the driver station to alerts
     * that the function doesn't work with April Tags.
     */
    double GetTargetSkew() override;

    /**
     * Get the alternate horizontal distance (related to the target ambiguity) between the robot and the target
     * Needs the camera to be calibrated in PhotonVision dashboard and
     * cameraToRobot to be initialised.
     * 
     * @return the alternate horizontal distance between the robot and the target or -1.0
     * if there is no currently analysed target
     */
    double AlternateHorizontalDistanceToTarget();

    /**
     * Get the alternative estimated target position on the field (related to the target ambiguity).
     * Needs the camera to be correctly calibrated in PhotonVision dashboard
     * and cameraToRobot and pRobotPos to be initialised
     * 
     * @return the alternate estimated target position on the field or the public ErrorPose2dOutput
     * if there is no currently analysed target (or a needed parameter not initialised)
     */
    frc::Pose2d GetAlternativeTargetPose();
};