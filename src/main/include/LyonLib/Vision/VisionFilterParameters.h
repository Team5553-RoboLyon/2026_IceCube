/*******************************************************************************
 * 
 * File        : VisionFilterParameters.h (v1.0)
 * Library     : LyonLib (from 2026)
 * Description : Parameters for filtering vision measurements. This struct 
 *               encapsulates all the tunable parameters used in the vision 
 *               processing pipeline.
 * 
 * Authors     : AKA (2026) and inspired by Team 4481
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>

/**
 * Parameters to filter vision measurements.
 *
 * @param xyStandardDevBase The base value for the translational standard deviation.
 * @param rotStandardDevBase The base value for the rotational standard deviation.
 * @param aprilTagWidth The width of the AprilTag.
 * @param maxAmbiguityRatio The maximum ambiguity ratio for a target to be considered valid.
 * @param maxAprilTagDistance The maximum distance to an AprilTag for which a measurement will be considered (meters).
 * @param estimatedFOV Estimated horizontal field of view of camera as Rotation2d.
 * @param zMargin Margin in Z direction (height) outside which selected poses are not considered valid.
 * @param aprilTagFieldLayout Layout of the AprilTags around the field.
 */
struct VisionFilterParameters {
  double xyStandardDevBase;
  double rotStandardDevBase;
  units::meter_t aprilTagWidth;
  double maxAmbiguityRatio;
  double maxAprilTagDistance;
  frc::Rotation2d estimatedFOV;
  units::meter_t zMargin;
  frc::AprilTagFieldLayout aprilTagFieldLayout;
};