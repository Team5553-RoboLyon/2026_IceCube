#include "subsystems/turret/camera/PhotonCamera.h"
#include "frc/smartdashboard/SmartDashboard.h"

PhotonCamera::PhotonCamera(photon::PhotonCamera* pCamera) : m_pCamera(pCamera)
{};  

PhotonCamera::PhotonCamera(photon::PhotonCamera* pCamera, frc::Pose2d* pRobotPos, frc::Transform3d cameraToRobot) : m_pCamera(pCamera), m_pRobotPos(pRobotPos), m_cameraToTarget(cameraToRobot)
{};

bool PhotonCamera::HasTarget()
{
  return m_pipelineResult.HasTargets();  
}

void PhotonCamera::UpdateResults()
{
  std::vector<photon::PhotonPipelineResult> m_unreadResults = m_pCamera->GetAllUnreadResults();
  m_pipelineResult = m_unreadResults[m_unreadResults.size() - 1];
  m_targets = m_pipelineResult.GetTargets();
  m_bestTarget = m_targets.begin();
}

double PhotonCamera::GetTargetPitch()
{
  if (m_pipelineResult.HasTargets())
  {
    return m_bestTarget->GetPitch();
  }
  return 90.0;
}

double PhotonCamera::GetTargetYaw()
{
  if (m_pipelineResult.HasTargets())
  {
    return m_bestTarget->GetYaw();
  }
  return 90.0;
}

double PhotonCamera::GetTargetArea()
{
  if (m_pipelineResult.HasTargets())
  {
    return m_bestTarget->GetArea();
  }
  return 0.0;
}

double PhotonCamera::GetTargetSkew()
{
  if (m_pipelineResult.HasTargets())
  {
    return m_bestTarget->GetSkew();
  }
  return 181.0;
}

std::vector<photon::TargetCorner> PhotonCamera::GetTargetCorners() //TODO
{
  if (m_pipelineResult.HasTargets())
  {
    return m_bestTarget->GetDetectedCorners();
  }
  return ErrorCornersVectorOutput; 
}


double PhotonCamera::GetHorizontalDistanceToTarget()
{
  if (m_pipelineResult.HasTargets())
  {
    frc::Transform3d robotToTarget = m_bestTarget->GetBestCameraToTarget()+m_cameraToTarget;
    return sqrt(pow(robotToTarget.X().value(),2)+pow(robotToTarget.X().value(),2));
  }
  return -1.0;
}

frc::Pose2d PhotonCamera::GetTargetPose()
{
  if (m_pRobotPos != nullptr && HasTarget())
  {
    frc::Transform3d transform3dToObject = m_bestTarget->GetBestCameraToTarget()+m_cameraToTarget;
    frc::Transform2d transform2dToObject{transform3dToObject.X(), transform3dToObject.Y(), transform3dToObject.Rotation().Z()};
    return *m_pRobotPos + transform2dToObject;
  }
  DEBUG_ASSERT(false,
              "GetTargetPose() maybe used with m_pRobotPos not initialized (or there is no target)");
  return ErrorPose2dOutput;
}

void PhotonCamera::SetPipeline(int pipelineIndex)
{
  m_pCamera->SetPipelineIndex(pipelineIndex);
  UpdateResults();
}

bool PhotonCamera::NextTarget()
{
  if (m_bestTarget != m_targets.end() && m_bestTarget != m_targets.end()--)
  {
    m_bestTarget++;
    return true;
  }
  return false;
}

bool PhotonCamera::PrevTarget()
{
  if (m_bestTarget != m_targets.begin())
  {
    m_bestTarget--;
    return true;
  }
  return false;
}

units::second_t PhotonCamera::GetLatency()
{
  return m_pipelineResult.GetLatency();
}

