/*
 * QUESTNAV
   https://github.com/QuestNav
 * Copyright (C) 2025 QuestNav
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published.
 */

#include "questnav/QuestNav.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <units/time.h>

#include <fmt/format.h>

namespace questnav {

QuestNav::QuestNav() : m_ntInstance(nt::NetworkTableInstance::GetDefault()) {
  auto questNavTable = m_ntInstance.GetTable("QuestNav");

  // Initialize subscribers
  m_frameDataSubscriber = std::make_unique<nt::ProtobufSubscriber<
      questnav::protos::data::ProtobufQuestNavFrameData>>(
      questNavTable
          ->GetProtobufTopic<questnav::protos::data::ProtobufQuestNavFrameData>(
              "frameData")
          .Subscribe(questnav::protos::data::ProtobufQuestNavFrameData(),
                     {.periodic = 0.01,
                      .sendAll = true,
                      .pollStorage = 20}));

  m_deviceDataSubscriber = std::make_unique<nt::ProtobufSubscriber<
      questnav::protos::data::ProtobufQuestNavDeviceData>>(
      questNavTable
          ->GetProtobufTopic<
              questnav::protos::data::ProtobufQuestNavDeviceData>("deviceData")
          .Subscribe(questnav::protos::data::ProtobufQuestNavDeviceData()));

  m_responseSubscriber = std::make_unique<nt::ProtobufSubscriber<
      questnav::protos::commands::ProtobufQuestNavCommandResponse>>(
      questNavTable
          ->GetProtobufTopic<
              questnav::protos::commands::ProtobufQuestNavCommandResponse>(
              "response")
          .Subscribe(
              questnav::protos::commands::ProtobufQuestNavCommandResponse(),
              {.periodic = 0.05, .sendAll = true, .pollStorage = 20}));

  m_versionSubscriber = std::make_unique<nt::StringSubscriber>(
      questNavTable->GetStringTopic("version").Subscribe("unknown"));

  // Initialize publisher
  m_requestPublisher = std::make_unique<nt::ProtobufPublisher<
      questnav::protos::commands::ProtobufQuestNavCommand>>(
      questNavTable
          ->GetProtobufTopic<questnav::protos::commands::ProtobufQuestNavCommand>(
              "request")
          .Publish());
}

QuestNav::~QuestNav() = default;

void QuestNav::SetPose(const frc::Pose3d& pose) {
  questnav::protos::commands::ProtobufQuestNavCommand command;
  command.set_type(questnav::protos::commands::POSE_RESET);
  command.set_command_id(++m_lastSentRequestId);

  auto* payload = command.mutable_pose_reset_payload();
  auto* targetPose = payload->mutable_target_pose();

  // Convert frc::Pose3d to protobuf
  auto translation = pose.Translation();
  auto* protoTranslation = targetPose->mutable_translation();
  protoTranslation->set_x(translation.X().value());
  protoTranslation->set_y(translation.Y().value());
  protoTranslation->set_z(translation.Z().value());

  auto rotation = pose.Rotation();
  auto quaternion = rotation.GetQuaternion();
  auto* protoRotation = targetPose->mutable_rotation();
  auto* protoQuaternion = protoRotation->mutable_q();
  protoQuaternion->set_w(quaternion.W());
  protoQuaternion->set_x(quaternion.X());
  protoQuaternion->set_y(quaternion.Y());
  protoQuaternion->set_z(quaternion.Z());

  m_requestPublisher->Set(command);
}

void QuestNav::SetPose(const frc::Pose2d& pose) {
  SetPose(frc::Pose3d(pose));
}

std::optional<int> QuestNav::GetBatteryPercent() const {
  auto deviceData = m_deviceDataSubscriber->Get();
  if (deviceData) {
    return deviceData->battery_percent();
  }
  return std::nullopt;
}

std::optional<int> QuestNav::GetFrameCount() const {
  auto frameData = m_frameDataSubscriber->Get();
  if (frameData) {
    return frameData->frame_count();
  }
  return std::nullopt;
}

std::optional<int> QuestNav::GetTrackingLostCounter() const {
  auto deviceData = m_deviceDataSubscriber->Get();
  if (deviceData) {
    return deviceData->tracking_lost_counter();
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
  if (frameData) {
    return frameData->timestamp();
  }
  return std::nullopt;
}

bool QuestNav::IsTracking() const {
  auto frameData = m_frameDataSubscriber->Get();
  if (frameData) {
    return frameData->istracking();
  }
  return false;  // Return false if no data for failsafe
}

std::vector<PoseFrame> QuestNav::GetAllUnreadPoseFrames() {
  std::vector<PoseFrame> result;
  auto frameDataArray = m_frameDataSubscriber->ReadQueue();

  for (const auto& timestampedData : frameDataArray) {
    const auto& frameData = timestampedData.value;

    // Convert protobuf pose to frc::Pose3d
    const auto& protoPose = frameData.pose3d();
    const auto& protoTranslation = protoPose.translation();
    const auto& protoRotation = protoPose.rotation();
    const auto& protoQuaternion = protoRotation.q();

    frc::Translation3d translation{units::meter_t{protoTranslation.x()},
                                   units::meter_t{protoTranslation.y()},
                                   units::meter_t{protoTranslation.z()}};

    frc::Rotation3d rotation{frc::Quaternion{
        protoQuaternion.w(), protoQuaternion.x(), protoQuaternion.y(),
        protoQuaternion.z()}};

    frc::Pose3d pose{translation, rotation};

    // Convert server timestamp from microseconds to seconds
    double dataTimestamp =
        units::second_t(units::microsecond_t(timestampedData.serverTime))
            .value();

    result.emplace_back(pose, dataTimestamp, frameData.timestamp(),
                        frameData.frame_count(), frameData.istracking());
  }

  return result;
}

void QuestNav::CommandPeriodic() {
  CheckVersionMatch();

  auto responses = m_responseSubscriber->ReadQueue();
  for (const auto& timestampedResponse : responses) {
    const auto& response = timestampedResponse.value;
    if (!response.success()) {
      frc::DriverStation::ReportError(
          fmt::format("QuestNav command failed!\n{}", response.error_message()));
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
