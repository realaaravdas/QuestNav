#include "questnav/QuestNav.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <units/time.h>

#include <fmt/format.h>

namespace questnav {

QuestNav::QuestNav() : m_ntInstance(nt::NetworkTableInstance::GetDefault()) {
  auto questNavTable = m_ntInstance.GetTable("QuestNav");

  // Initialize subscribers
  m_frameDataSubscriber = std::make_unique<nt::StructSubscriber<questnav::QuestNavFrameData>>(
      questNavTable
          ->GetStructTopic<questnav::QuestNavFrameData>("frameData")
          .Subscribe(questnav::QuestNavFrameData{},
                     {.periodic = 0.01,
                      .sendAll = true,
                      .pollStorage = 20}));

  m_deviceDataSubscriber = std::make_unique<nt::StructSubscriber<questnav::QuestNavDeviceData>>(
      questNavTable
          ->GetStructTopic<questnav::QuestNavDeviceData>("deviceData")
          .Subscribe(questnav::QuestNavDeviceData{}));

  m_responseSubscriber = std::make_unique<nt::StructSubscriber<questnav::QuestNavCommandResponse>>(
      questNavTable
          ->GetStructTopic<questnav::QuestNavCommandResponse>("response")
          .Subscribe(
              questnav::QuestNavCommandResponse{},
              {.periodic = 0.05, .sendAll = true, .pollStorage = 20}));

  m_versionSubscriber = std::make_unique<nt::StringSubscriber>(
      questNavTable->GetStringTopic("version").Subscribe("unknown"));

  // Initialize publisher
  m_requestPublisher = std::make_unique<nt::StructPublisher<questnav::QuestNavCommand>>(
      questNavTable
          ->GetStructTopic<questnav::QuestNavCommand>("request")
          .Publish());
}

QuestNav::~QuestNav() = default;

void QuestNav::SetPose(const frc::Pose3d& pose) {
  questnav::QuestNavCommand command;
  command.type = questnav::QuestNavCommandType::PoseReset;
  command.commandId = ++m_lastSentRequestId;
  command.targetPose = pose;

  m_requestPublisher->Set(command);
}

void QuestNav::SetPose(const frc::Pose2d& pose) {
  SetPose(frc::Pose3d(pose));
}

std::optional<double> QuestNav::GetBatteryPercent() const {
  auto deviceData = m_deviceDataSubscriber->Get();
  // Check if data is fresh enough or valid (simplest check is just availability in topic)
  // For struct subscriber, Get() returns the value or default. 
  // To check existence we might want to check timestamp, but strict optional isn't direct
  // relying on topic existence check or default values.
  // Here we return the value if the topic exists and has value.
  if (m_deviceDataSubscriber->GetTopic().Exists()) {
      return deviceData.batteryPercent;
  }
  return std::nullopt;
}

std::optional<int> QuestNav::GetFrameCount() const {
  auto frameData = m_frameDataSubscriber->Get();
  if (m_frameDataSubscriber->GetTopic().Exists()) {
    return frameData.frameCount;
  }
  return std::nullopt;
}

std::optional<int> QuestNav::GetTrackingLostCounter() const {
  auto deviceData = m_deviceDataSubscriber->Get();
  if (m_deviceDataSubscriber->GetTopic().Exists()) {
    return deviceData.trackingLostCounter;
  }
  return std::nullopt;
}

bool QuestNav::IsConnected() const {
  auto currentTime = frc::Timer::GetFPGATimestamp();
  auto lastChange =
      units::microsecond_t(m_frameDataSubscriber->GetLastChange());
  auto timeSinceLastUpdate = currentTime - lastChange;

  return timeSinceLastUpdate < units::millisecond_t(kConnectionTimeoutMs);
}

double QuestNav::GetLatency() const {
  auto currentTime = frc::Timer::GetFPGATimestamp();
  auto lastChange =
      units::microsecond_t(m_frameDataSubscriber->GetLastChange());
  auto latency = currentTime - lastChange;

  return units::millisecond_t(latency).value();
}

std::optional<double> QuestNav::GetAppTimestamp() const {
  auto frameData = m_frameDataSubscriber->Get();
  if (m_frameDataSubscriber->GetTopic().Exists()) {
    return frameData.timestamp;
  }
  return std::nullopt;
}

bool QuestNav::IsTracking() const {
  auto frameData = m_frameDataSubscriber->Get();
  if (m_frameDataSubscriber->GetTopic().Exists()) {
    return frameData.isTracking;
  }
  return false;  // Return false if no data for failsafe
}

std::vector<PoseFrame> QuestNav::GetAllUnreadPoseFrames() {
  std::vector<PoseFrame> result;
  auto frameDataArray = m_frameDataSubscriber->ReadQueue();

  for (const auto& timestampedData : frameDataArray) {
    const auto& frameData = timestampedData.value;

    // Direct struct access, no protobuf conversion needed
    frc::Pose3d pose = frameData.pose;

    // Convert server timestamp from microseconds to seconds
    double dataTimestamp =
        units::second_t(units::microsecond_t(timestampedData.serverTime))
            .value();

    result.emplace_back(pose, dataTimestamp, frameData.timestamp,
                        frameData.frameCount, frameData.isTracking);
  }

  return result;
}

void QuestNav::CommandPeriodic() {
  CheckVersionMatch();

  auto responses = m_responseSubscriber->ReadQueue();
  for (const auto& timestampedResponse : responses) {
    const auto& response = timestampedResponse.value;
    if (!response.success) {
      frc::DriverStation::ReportError(
          fmt::format("QuestNav command failed!\n{}", response.errorMessage));
    }
  }
}

std::string QuestNav::GetLibVersion() const {
  // This should match the version in build.gradle
  // For now, return a placeholder. This could be set via CMake define
  return "2025-1.0.0-dev";
}

std::string QuestNav::GetQuestNavVersion() const {
  return m_versionSubscriber->Get();
}

void QuestNav::SetVersionCheckEnabled(bool enabled) {
  m_versionCheckEnabled = enabled;
}

void QuestNav::CheckVersionMatch() {
  if (!m_versionCheckEnabled || !IsConnected()) {
    return;
  }

  auto currentTime = frc::Timer::GetFPGATimestamp().value();
  if ((currentTime - m_lastVersionCheckTime) < kVersionCheckIntervalSeconds) {
    return;
  }
  m_lastVersionCheckTime = currentTime;

  auto libVersion = GetLibVersion();
  auto questNavVersion = GetQuestNavVersion();

  if (questNavVersion != libVersion) {
    frc::DriverStation::ReportWarning(fmt::format(
        "WARNING FROM QUESTNAV: QuestNavLib version ({}) on your robot does "
        "not match QuestNav app version ({}) on your headset. "
        "This may cause compatibility issues. Check the version of your "
        "vendordep and the app running on your headset.",
        libVersion, questNavVersion));
  }
}

}  // namespace questnav
