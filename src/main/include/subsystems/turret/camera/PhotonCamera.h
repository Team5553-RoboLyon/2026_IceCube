#pragma once

#include "photon/PhotonCamera.h"
#include "frc/geometry/Pose2d.h"
#include "wpi/SmallVector.h"

#include <span>
#include <string>

#include "Lyonlib/logging/DebugUtils.h"

/**
 * This class can be used to handle a camera using PhotonVision. It handles
 * getting and keeping the camera data and provides a simple acces to all this
 * data to simplify the robot code.
 */
class PhotonCamera
{
    public :

    /**
     * Construct an instance of a camera using PhotonVision
     * 
     * @param pCamera A pointer that points at the PhotonVision camera
     */
    PhotonCamera(photon::PhotonCamera* pCamera);

    /**
     * Construct an instance of a camera using PhotonVision
     * 
     * @param pCamera A pointer that points at the PhotonVision camera
     * @param pRobotPos A pointer that points to a frc::Pose2d representing the
     *                  robot pos. It is used in some functions related to position
     *                  of the field like GetTargetPos()
     * @param cameraToRobot
     */
    PhotonCamera(photon::PhotonCamera* pCamera, frc::Pose2d* pRobotPos, frc::Transform3d cameraToRobot);

    ~PhotonCamera() = default;

    /**
     * Update the results and the targets data to the latest camera frame
     * It also reset the m_bestTarget iterator to the first target in the 
     * target list (if there is targets)
     */
    void UpdateResults();

    /**
     * Returns if there is one or more targets in the actual analysed camera
     * frame
     * 
     * @return Returns if there is one or more targets in the actual analysed camera
     * frame
     */
    bool HasTarget();

    /**
     * Get the Pitch angle (positive-up) of the currently analysed target
     * 
     * @return the Pitch angle (positive-up) of the current analysed target or 90.0
     * if there is no target currently analised
     */
    double GetTargetPitch();

    /**
     * Get the Yaw angle (positive-left) of the currently analysed target
     * 
     * @return the Yaw angle (positive-left) of the current analysed target or 90.0
     * if there is no target currently analised
     */
    double GetTargetYaw();

    /**
     * Get the area (in percentage of frame) of the currenly analysed target
     * 
     * @return the area (in percentage of frame) of the current analysed target or 0.0
     * if there is no target currently analised
     */
    double GetTargetArea();

    /**
     * Get the skew (counter-clockwise positive) of the currently analysed target
     * 
     * @return the skew (counter-clockwise positive) of the current analysed target or 181.0
     * if there is no target currently analised
     */
    virtual double GetTargetSkew();

    /**
     * Get a vector conataining data about the currently analysed target corners
     * 
     * @return a vector conataining data about the current analysed target corners
     * or the public ErrorCornersVectorOutput if there is no currently analysed target
     * (or a needed parameter not initialised)
     */
    std::vector<photon::TargetCorner> GetTargetCorners();

    /**
     * Get the horizontal distance between the robot and the target
     * Needs the camera to be calibrated in PhotonVision dashboard and
     * cameraToRobot to be initialised.
     * 
     * @return the horizontal distance between the robot and the target or -1.0
     * if there is no currently analysed target
     */
    double GetHorizontalDistanceToTarget();

    /**
     * Get the estimated target position on the field.
     * Needs the camera to be correctly calibrated in PhotonVision dashboard
     * and cameraToRobot and pRobotPos to be initialised
     * 
     * @return the estimated target position on the field or the public ErrorPose2dOutput
     * if there is no currently analysed target (or a needed parameter not initialised)
     */
    frc::Pose2d GetTargetPose();

    /**
     * This function allows you to change the pipeline used by the camera
     * When used it will automatically run UpdateResults() to actualise data 
     * with the new pipeline
     */
    void SetPipeline(int pipelineIndex);

    /**
     * This function allows to change the analysed target passing to
     * the next one (if there is a next one) in the target list sorted
     * respecting the parameters choosen in PhotonVision dashboard
     * 
     * @return true if the target have been changed or false if it
     * there is no next target
     */
    bool NextTarget();

    /**
     * This function allows to change the analysed target passing to
     * the previous one (if there is a preivious one) in the target list sorted
     * respecting the parameters choosen in PhotonVision dashboard
     * 
     * @return true if the target have been changed or false if it
     * there is no previous target
     */
    bool PrevTarget();

    /**
     * Get the latency of the pipeline
     * 
     * @return the latency of the pipeline
     */
    units::second_t GetLatency();

    frc::Pose2d ErrorPose2dOutput {-1.0_m, 0.0_m, 0_deg};
    std::vector<photon::TargetCorner> ErrorCornersVectorOutput{};

    protected :

    photon::PhotonCamera* m_pCamera;
    photon::PhotonPipelineResult m_pipelineResult;
    std::span<const photon::PhotonTrackedTarget> m_targets;
    std::span<const photon::PhotonTrackedTarget>::iterator m_bestTarget;
    frc::Pose2d* m_pRobotPos;
    frc::Transform3d m_cameraToTarget;
    double m_horizontalFOV;
    double m_verticalFOV;

};