/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2026 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
package gg.questnav.questnav.structs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class QuestNavPoseResetPayload implements StructSerializable {
  public Pose3d targetPose;

  public QuestNavPoseResetPayload(Pose3d targetPose) {
    this.targetPose = targetPose;
  }

  public QuestNavPoseResetPayload() {
    this.targetPose = new Pose3d();
  }

  public static final Struct<QuestNavPoseResetPayload> struct =
      new Struct<>() {
        @Override
        public Class<QuestNavPoseResetPayload> getTypeClass() {
          return QuestNavPoseResetPayload.class;
        }

        @Override
        public String getTypeName() {
          return "QuestNavPoseResetPayload";
        }

        @Override
        public String getTypeString() {
          return "struct:QuestNavPoseResetPayload";
        }

        @Override
        public int getSize() {
          return Pose3d.struct.getSize();
        }

        @Override
        public String getSchema() {
          return "Pose3d targetPose";
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {Pose3d.struct};
        }

        @Override
        public QuestNavPoseResetPayload unpack(ByteBuffer bb) {
          return new QuestNavPoseResetPayload(Pose3d.struct.unpack(bb));
        }

        @Override
        public void pack(ByteBuffer bb, QuestNavPoseResetPayload value) {
          Pose3d.struct.pack(bb, value.targetPose);
        }
      };
}
