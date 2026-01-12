/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2026 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
package gg.questnav.questnav.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class QuestNavCommand implements StructSerializable {
  public QuestNavCommandType type;
  public long commandId;
  public QuestNavPoseResetPayload poseResetPayload;

  public QuestNavCommand(
      QuestNavCommandType type, long commandId, QuestNavPoseResetPayload poseResetPayload) {
    this.type = type;
    this.commandId = commandId;
    this.poseResetPayload = poseResetPayload;
  }

  public QuestNavCommand() {
    this.type = QuestNavCommandType.UNSPECIFIED;
    this.commandId = 0;
    this.poseResetPayload = new QuestNavPoseResetPayload();
  }

  public static final Struct<QuestNavCommand> struct =
      new Struct<>() {
        @Override
        public Class<QuestNavCommand> getTypeClass() {
          return QuestNavCommand.class;
        }

        @Override
        public String getTypeName() {
          return "QuestNavCommand";
        }

        @Override
        public String getTypeString() {
          return "struct:QuestNavCommand";
        }

        @Override
        public int getSize() {
          return Struct.kSizeInt8 + Struct.kSizeInt64 + QuestNavPoseResetPayload.struct.getSize();
        }

        @Override
        public String getSchema() {
          return "uint8 type;int64 commandId;QuestNavPoseResetPayload poseResetPayload";
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {QuestNavPoseResetPayload.struct};
        }

        @Override
        public QuestNavCommand unpack(ByteBuffer bb) {
          // Unsigned byte handled as int/short in Java, but here we just read byte
          int typeValue = bb.get() & 0xFF;
          QuestNavCommandType type = QuestNavCommandType.fromValue(typeValue);
          long commandId = bb.getLong();
          QuestNavPoseResetPayload poseResetPayload = QuestNavPoseResetPayload.struct.unpack(bb);
          return new QuestNavCommand(type, commandId, poseResetPayload);
        }

        @Override
        public void pack(ByteBuffer bb, QuestNavCommand value) {
          bb.put((byte) value.type.getValue());
          bb.putLong(value.commandId);
          QuestNavPoseResetPayload.struct.pack(bb, value.poseResetPayload);
        }
      };
}
