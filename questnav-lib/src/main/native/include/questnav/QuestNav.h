/*
 * QUESTNAV
   https://github.com/QuestNav
 * Copyright (C) 2025 QuestNav
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published.
 */

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/ProtobufTopic.h>
#include <networktables/StringTopic.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "commands.pb.h"
#include "data.pb.h"
#include "geometry3d.pb.h"
#include "questnav/PoseFrame.h"

namespace questnav {

/**
 * The QuestNav class provides a comprehensive interface for communicating with
 * an Oculus/Meta Quest VR headset for robot localization and tracking in FRC
 * robotics applications.
 *
 * This class handles all aspects of Quest-robot communication including:
 * - Real-time pose tracking and localization data
 * - Command sending and response handling
 * - Device status monitoring (battery, tracking state, connectivity)
 * - NetworkTables-based communication protocol
 *
 * Basic Usage:
 *
 * \code{.cpp}
 * // Create QuestNav instance
 * questnav::QuestNav questNav;
 *
 * // Set initial robot pose (required for field-relative tracking)
 * frc::Pose2d initialPose{1.0_m, 2.0_m, frc::Rotation2d(90_deg)};
 * questNav.SetPose(initialPose);
 *
 * // In robot periodic methods
 * void RobotPeriodic() {
 *   questNav.CommandPeriodic(); // Process command responses
 *
 *   // Get latest pose data
 *   auto newFrames = questNav.GetAllUnreadPoseFrames();
 *   for (const auto& frame : newFrames) {
 *     // Use frame.questPose3d and frame.dataTimestamp with pose estimator
 *   }
 *
 *   // Monitor connection and device status
 *   if (questNav.IsConnected() && questNav.IsTracking()) {
 *     // Quest is connected and tracking - safe to use pose data
 *   }
 * }
 * \endcode
 *
 * Coordinate Systems:
 *
 * QuestNav uses the WPILib field coordinate system:
 * - X-axis: Forward direction (towards opposing alliance)
 * - Y-axis: Left direction (when facing forward)
 * - Rotation: Counter-clockwise positive (standard mathematical convention)
 * - Units: Meters for translation, radians for rotation
 *
 * Threading and Performance:
 *
 * This class is designed for use in FRC robot code and follows WPILib threading
 * conventions:
 * - All methods are thread-safe for typical FRC usage patterns
 * - NetworkTables handles the underlying communication asynchronously
 * - Call CommandPeriodic() regularly to process command responses
 */
class QuestNav {
 public:
  /**
   * Creates a new QuestNav instance for communicating with a Quest headset.
   *
   * This constructor initializes all necessary NetworkTables subscribers and
   * publishers for communication with the Quest device. The instance is ready
   * to use immediately, but you should call SetPose() to establish
   * field-relative tracking before relying on pose data.
   */
  QuestNav();

  /**
   * Destructor.
   */
  ~QuestNav();

  /**
   * Sets the field-relative pose of the Quest headset by commanding it to
   * reset its tracking.
   *
   * This method sends a pose reset command to the Quest headset, telling it
   * where the Quest is currently located on the field. This is essential for
   * establishing field-relative tracking and should be called:
   * - At the start of autonomous or teleop when the Quest position is known
   * - When the robot (and Quest) is placed at a known location (e.g., against
   *   field walls)
   * - After significant tracking drift is detected
   * - When integrating with other localization systems (vision, odometry)
   *
   * Important: This should be the Quest's pose, not the robot's pose. If you
   * know the robot's pose, you need to apply the mounting offset to get the
   * Quest's pose before calling this method.
   *
   * The command is sent asynchronously. Monitor command success/failure by
   * calling CommandPeriodic() regularly, which will log any errors to the
   * DriverStation.
   *
   * @param pose The Quest's current field-relative pose in WPILib coordinates
   *             (meters for translation, radians for rotation)
   */
  void SetPose(const frc::Pose3d& pose);

  /**
   * Convenience method to set pose from a 2D pose.
   * Converts to 3D with Z=0 and no pitch/roll.
   *
   * @param pose The Quest's current field-relative 2D pose
   */
  void SetPose(const frc::Pose2d& pose);

  /**
   * Returns the Quest headset's current battery level as a percentage.
   *
   * This method provides real-time battery status information from the Quest
   * device, which is useful for:
   * - Monitoring device health during matches
   * - Implementing low-battery warnings or behaviors
   * - Planning charging schedules between matches
   * - Triggering graceful shutdown procedures when battery is critical
   *
   * Battery level guidelines:
   * - 80-100%: Excellent - full match capability
   * - 50-80%: Good - normal operation expected
   * - 20-50%: Fair - consider charging after match
   * - 10-20%: Low - charge soon, monitor closely
   * - 0-10%: Critical - immediate charging required
   *
   * @return std::optional containing the battery percentage (0-100), or
   *         std::nullopt if no device data is available or Quest is
   *         disconnected
   */
  std::optional<int> GetBatteryPercent() const;

  /**
   * Gets the current frame count from the Quest headset.
   *
   * @return std::optional containing the frame count value, or std::nullopt if
   *         no frame data is available
   */
  std::optional<int> GetFrameCount() const;

  /**
   * Gets the number of tracking lost events since the Quest connected to the
   * robot.
   *
   * @return std::optional containing the tracking lost counter value, or
   *         std::nullopt if no device data is available
   */
  std::optional<int> GetTrackingLostCounter() const;

  /**
   * Determines if the Quest headset is currently connected to the robot.
   * Connection is determined by how stale the last received frame from the
   * Quest is.
   *
   * @return true if the Quest is connected, false otherwise
   */
  bool IsConnected() const;

  /**
   * Gets the latency of the Quest > Robot Connection. Returns the latency
   * between the current time and the last frame data update.
   *
   * @return The latency in milliseconds
   */
  double GetLatency() const;

  /**
   * Returns the Quest app's uptime timestamp for debugging and diagnostics.
   *
   * Important: For integration with a pose estimator, use the timestamp from
   * PoseFrame::dataTimestamp instead! This method provides the Quest's
   * internal application timestamp, which is useful for:
   * - Debugging timing issues between Quest and robot
   * - Calculating Quest-side processing latency
   * - Monitoring Quest application uptime
   * - Correlating with Quest-side logs
   *
   * @return std::optional containing the Quest app uptime in seconds, or
   *         std::nullopt if no frame data is available
   */
  std::optional<double> GetAppTimestamp() const;

  /**
   * Gets the current tracking state of the Quest headset.
   *
   * This method indicates whether the Quest's visual-inertial tracking system
   * is currently functioning and providing reliable pose data. Tracking can be
   * lost due to:
   * - Poor lighting conditions (too dark or too bright)
   * - Lack of visual features in the environment
   * - Rapid motion or high acceleration
   * - Camera occlusion or obstruction
   * - Hardware issues or overheating
   *
   * Important: When tracking is lost, pose data becomes unreliable and should
   * not be used for robot control. Implement fallback localization methods
   * (wheel odometry, vision, etc.) for when Quest tracking is unavailable.
   *
   * @return true if the Quest is actively tracking and pose data is reliable,
   *         false if tracking is lost or no device data is available
   */
  bool IsTracking() const;

  /**
   * Retrieves all new pose frames received from the Quest since the last call
   * to this method.
   *
   * This is the primary method for integrating QuestNav with FRC pose
   * estimation systems. It returns a vector of PoseFrame objects containing
   * pose data and timestamps that can be fed directly into a pose estimator.
   *
   * Each frame contains:
   * - Pose data: Robot position and orientation in field coordinates
   * - NetworkTables timestamp: When the data was received (use this for pose
   *   estimation)
   * - App timestamp: Quest internal timestamp (for debugging only)
   * - Frame count: Sequential frame number for detecting drops
   *
   * Important: This method consumes the frame queue, so each frame is only
   * returned once. Call this method regularly (every robot loop) to avoid
   * missing frames.
   *
   * @return Vector of new PoseFrame objects received since the last call.
   *         Empty vector if no new frames are available or Quest is
   *         disconnected.
   */
  std::vector<PoseFrame> GetAllUnreadPoseFrames();

  /**
   * Processes command responses from the Quest headset and handles any errors.
   *
   * This method must be called regularly (typically in RobotPeriodic()) to:
   * - Process responses to commands sent via SetPose()
   * - Log command failures to the DriverStation for debugging
   * - Maintain proper command/response synchronization
   * - Prevent command response queue overflow
   *
   * The method automatically handles:
   * - Success responses: Silently acknowledged
   * - Error responses: Logged to DriverStation with error details
   * - Duplicate responses: Ignored to prevent spam
   * - Out-of-order responses: Handled gracefully
   *
   * Performance: This method is lightweight and safe to call every robot loop
   * (20ms). It only processes new responses and exits quickly when none are
   * available.
   */
  void CommandPeriodic();

  /**
   * Retrieves the QuestNav-lib version number.
   *
   * @return The version number as a string, or "0-0.0.0" if unable to retrieve
   */
  std::string GetLibVersion() const;

  /**
   * Retrieves the QuestNav app version number from the Quest headset.
   *
   * @return The version number as a string, or "unknown" if unable to retrieve
   */
  std::string GetQuestNavVersion() const;

  /**
   * Turns the version check on or off. When on, a warning will be reported to
   * the DriverStation if the QuestNavLib and QuestNav app versions do not
   * match.
   *
   * @param enabled true to enable version checking, false to disable it.
   *                Default is true.
   */
  void SetVersionCheckEnabled(bool enabled);

 private:
  // NetworkTables instance
  nt::NetworkTableInstance m_ntInstance;

  // NetworkTables topics
  std::unique_ptr<nt::ProtobufSubscriber<
      questnav::protos::data::ProtobufQuestNavFrameData>>
      m_frameDataSubscriber;
  std::unique_ptr<nt::ProtobufSubscriber<
      questnav::protos::data::ProtobufQuestNavDeviceData>>
      m_deviceDataSubscriber;
  std::unique_ptr<nt::ProtobufSubscriber<
      questnav::protos::commands::ProtobufQuestNavCommandResponse>>
      m_responseSubscriber;
  std::unique_ptr<nt::StringSubscriber> m_versionSubscriber;
  std::unique_ptr<nt::ProtobufPublisher<
      questnav::protos::commands::ProtobufQuestNavCommand>>
      m_requestPublisher;

  // State tracking
  int m_lastSentRequestId = 0;
  bool m_versionCheckEnabled = true;
  double m_lastVersionCheckTime = 0.0;

  // Helper methods
  void CheckVersionMatch();
  static constexpr double kVersionCheckIntervalSeconds = 5.0;
  static constexpr double kConnectionTimeoutMs = 50.0;
};

}  // namespace questnav
