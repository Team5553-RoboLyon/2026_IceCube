#pragma once

#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"

/**
 * This class can be used to handle robot localisation using PhotonVision MultiTag.
 */
class MultiTagLocalisationCamera
{
    public :

    /**
     * Construct an instance of a camera using PhotonVision MultiTag to locate the robot
     * 
     * @param pCamera A pointer that points at the PhotonVision camera
     * @param pPosEstimator A pointer that points at the PhotonPoseEstimator used for localisation
     */
    MultiTagLocalisationCamera(photon::PhotonCamera* pCamera, photon::PhotonPoseEstimator* pPosEstimator);

    /**
     * Update the estimated position of the robot with photonvision MultiTag if possible (if the camera has targets)
     * @return if the position had been actualise with the latest camera frame 
     */
    bool UpdatePos();

    /**
     * Get the latest estimated position of the robot. 
     * To actualise the returned value you must use UpdatePos().
     * @return the frc::Pose2d of the latest estimated position of the robot
     */
    frc::Pose2d GetRobotPos();

    private :

    photon::PhotonCamera* m_pCamera;
    photon::PhotonPoseEstimator* m_pPosEstimator;
    std::optional<photon::EstimatedRobotPose> estimatedPos;
};