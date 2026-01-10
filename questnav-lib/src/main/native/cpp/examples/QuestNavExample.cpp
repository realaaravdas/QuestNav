/*
 * QUESTNAV C++ Example
 * 
 * This example demonstrates how to integrate QuestNav with an FRC robot
 * for vision-based localization and pose estimation.
 */

#include <questnav/QuestNav.h>
#include <frc/TimedRobot.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/length.h>

class QuestNavExample : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // QuestNav is automatically initialized
    frc::SmartDashboard::PutString("QuestNav Status", "Initialized");
  }

  void RobotPeriodic() override {
    // IMPORTANT: Call this every loop to process command responses
    m_questNav.CommandPeriodic();

    // Update dashboard with connection status
    bool connected = m_questNav.IsConnected();
    bool tracking = m_questNav.IsTracking();
    
    frc::SmartDashboard::PutBoolean("Quest Connected", connected);
    frc::SmartDashboard::PutBoolean("Quest Tracking", tracking);

    // Display latency if connected
    if (connected) {
      double latency = m_questNav.GetLatency();
      frc::SmartDashboard::PutNumber("Quest Latency (ms)", latency);
    }

    // Display battery level
    auto battery = m_questNav.GetBatteryPercent();
    if (battery) {
      frc::SmartDashboard::PutNumber("Quest Battery %", *battery);
      
      // Warn if battery is low
      if (*battery < 20) {
        frc::SmartDashboard::PutString("QuestNav Status", "LOW BATTERY!");
      }
    }

    // Get and display Quest version
    std::string questVersion = m_questNav.GetQuestNavVersion();
    frc::SmartDashboard::PutString("Quest Version", questVersion);

    // Process new pose frames
    ProcessPoseFrames();
  }

  void AutonomousInit() override {
    // Set initial robot pose when you know where the robot is
    // This example assumes robot starts at (1.5m, 2.0m) facing forward
    frc::Pose2d initialPose{1.5_m, 2.0_m, frc::Rotation2d{0_deg}};
    
    // IMPORTANT: This sets the Quest's pose, not the robot's
    // If Quest is mounted with an offset, you need to account for that
    m_questNav.SetPose(initialPose);
    
    frc::SmartDashboard::PutString("QuestNav Status", "Pose Reset");
  }

  void AutonomousPeriodic() override {
    // Your autonomous code here
    // Use pose data from ProcessPoseFrames() for navigation
  }

  void TeleopInit() override {
    // Optionally reset pose at teleop start if you know robot position
    // For example, if robot is against a field wall
  }

  void TeleopPeriodic() override {
    // Your teleop code here
    // Pose data is automatically processed in RobotPeriodic
  }

  void DisabledInit() override {
    frc::SmartDashboard::PutString("QuestNav Status", "Disabled");
  }

 private:
  void ProcessPoseFrames() {
    // Get all new pose frames since last call
    auto frames = m_questNav.GetAllUnreadPoseFrames();

    // Display frame count
    frc::SmartDashboard::PutNumber("Quest Frame Count", frames.size());

    // Process each frame
    for (const auto& frame : frames) {
      // Only use data when Quest is tracking and connected
      if (m_questNav.IsTracking() && m_questNav.IsConnected()) {
        // Convert 3D pose to 2D for most FRC applications
        auto pose2d = frame.questPose3d.ToPose2d();

        // Display current pose
        frc::SmartDashboard::PutNumber("Quest X", pose2d.X().value());
        frc::SmartDashboard::PutNumber("Quest Y", pose2d.Y().value());
        frc::SmartDashboard::PutNumber("Quest Rotation",
                                        pose2d.Rotation().Degrees().value());

        // Display frame information
        frc::SmartDashboard::PutNumber("Quest Frame #", frame.frameCount);
        frc::SmartDashboard::PutNumber("Quest Data Timestamp",
                                        frame.dataTimestamp);

        /*
         * In a real robot, you would add this to your pose estimator:
         *
         * // Standard deviations for vision measurements (tune these!)
         * wpi::array<double, 3> stdDevs = {0.1, 0.1, 0.05}; // x, y, rotation
         *
         * m_poseEstimator.AddVisionMeasurement(
         *     pose2d,
         *     units::second_t{frame.dataTimestamp},
         *     stdDevs
         * );
         */
      } else {
        // Quest lost tracking or disconnected
        frc::SmartDashboard::PutString("QuestNav Status", "Not Tracking");
      }
    }
  }

  // QuestNav instance - handles all communication with Quest headset
  questnav::QuestNav m_questNav;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<QuestNavExample>();
}
#endif
