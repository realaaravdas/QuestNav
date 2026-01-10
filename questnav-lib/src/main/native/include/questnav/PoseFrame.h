/*
 * QUESTNAV
   https://github.com/QuestNav
 * Copyright (C) 2025 QuestNav
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published.
 */

#pragma once

#include <frc/geometry/Pose3d.h>

namespace questnav {

/**
 * Represents a single frame of pose tracking data received from the Quest
 * headset.
 *
 * This struct encapsulates all the information needed to integrate Quest
 * tracking data with FRC pose estimation systems. Each frame represents a
 * single tracking measurement from the Quest's visual-inertial odometry
 * system.
 *
 * Usage with Pose Estimators:
 *
 * This struct is designed to work seamlessly with WPILib's pose estimation
 * framework:
 *
 * \code{.cpp}
 * auto frames = questNav.GetAllUnreadPoseFrames();
 * for (const auto& frame : frames) {
 *   if (questNav.IsTracking()) {
 *     poseEstimator.AddVisionMeasurement(
 *       frame.questPose3d,      // Use the pose measurement
 *       frame.dataTimestamp,    // Use the NetworkTables timestamp
 *       standardDeviations      // Your measurement uncertainty
 *     );
 *   }
 * }
 * \endcode
 *
 * Timestamp Usage:
 *
 * Two timestamps are provided for different use cases:
 * - dataTimestamp: NetworkTables reception time - use this for pose estimation
 * - appTimestamp: Quest internal time - use only for debugging/diagnostics
 *
 * Coordinate System:
 *
 * The pose data follows WPILib field coordinate conventions:
 * - X-axis: Forward (towards opposing alliance)
 * - Y-axis: Left (when facing forward)
 * - Rotation: Counter-clockwise positive
 * - Units: Meters for translation, radians for rotation
 */
struct PoseFrame {
  /**
   * The robot's pose on the field as measured by the Quest tracking system.
   * This will only provide meaningful field-relative coordinates after
   * QuestNav::SetPose() has been called to establish the field reference frame.
   */
  frc::Pose3d questPose3d;

  /**
   * The NetworkTables timestamp indicating when this frame data was received by
   * the robot. This timestamp should be used when adding vision measurements to
   * pose estimators as it represents when the measurement was available to the
   * robot code. Units: seconds since robot program start.
   */
  double dataTimestamp;

  /**
   * The Quest application's internal timestamp indicating when this frame was
   * generated. This is primarily useful for debugging timing issues and
   * calculating Quest-side latency. For pose estimation, use dataTimestamp
   * instead. Units: seconds since Quest app startup.
   */
  double appTimestamp;

  /**
   * Sequential frame number from the Quest tracking system. This counter
   * increments with each tracking frame and can be used to detect dropped
   * frames or measure effective frame rate. Resets to 0 when the Quest app
   * restarts.
   */
  int frameCount;

  /**
   * Indicates whether the Quest is currently tracking its position.
   */
  bool isTracking;

  /**
   * Construct a new PoseFrame object.
   *
   * @param questPose3d The 3D pose measurement
   * @param dataTimestamp NetworkTables timestamp (seconds)
   * @param appTimestamp Quest app internal timestamp (seconds)
   * @param frameCount Frame sequence number
   * @param isTracking Whether Quest is currently tracking
   */
  PoseFrame(frc::Pose3d questPose3d, double dataTimestamp, double appTimestamp,
            int frameCount, bool isTracking)
      : questPose3d(questPose3d),
        dataTimestamp(dataTimestamp),
        appTimestamp(appTimestamp),
        frameCount(frameCount),
        isTracking(isTracking) {}

  /**
   * Default constructor.
   */
  PoseFrame() = default;
};

}  // namespace questnav
