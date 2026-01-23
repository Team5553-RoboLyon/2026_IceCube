#include "subsystems/vision/MultiTagLocalisationCamera.h"

#include "Lyonlib/logging/DebugUtils.h"

MultiTagLocalisationCamera::MultiTagLocalisationCamera(photon::PhotonCamera* pCamera, photon::PhotonPoseEstimator* pPosEstimator) : m_pCamera(pCamera), m_pPosEstimator(pPosEstimator)
{}

bool MultiTagLocalisationCamera::UpdatePos()
{
    std::vector<photon::PhotonPipelineResult> m_UnreadResults = m_pCamera->GetAllUnreadResults();
    photon::PhotonPipelineResult cameraResults = m_UnreadResults[m_UnreadResults.size()-1];
    estimatedPos = m_pPosEstimator->EstimateCoprocMultiTagPose(cameraResults);
    return false;
}

frc::Pose2d MultiTagLocalisationCamera::GetRobotPos()
{
    return estimatedPos->estimatedPose.ToPose2d();
}