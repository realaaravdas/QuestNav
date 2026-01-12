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
import java.nio.charset.StandardCharsets;

public class QuestNavCommandResponse implements StructSerializable {
  public long commandId;
  public boolean success;
  public String errorMessage;

  public QuestNavCommandResponse(long commandId, boolean success, String errorMessage) {
    this.commandId = commandId;
    this.success = success;
    this.errorMessage = errorMessage;
  }

  public static final Struct<QuestNavCommandResponse> struct =
      new Struct<>() {
        @Override
        public Class<QuestNavCommandResponse> getTypeClass() {
          return QuestNavCommandResponse.class;
        }

        @Override
        public String getTypeName() {
          return "QuestNavCommandResponse";
        }

        @Override
        public String getTypeString() {
          return "struct:QuestNavCommandResponse";
        }

        @Override
        public int getSize() {
          return Struct.kSizeInt64 + Struct.kSizeBool + 256;
        }

        @Override
        public String getSchema() {
          return "int64 commandId;bool success;char errorMessage[256]";
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {};
        }

        @Override
        public QuestNavCommandResponse unpack(ByteBuffer bb) {
          long commandId = bb.getLong();
          boolean success = bb.get() != 0;

          byte[] strBytes = new byte[256];
          bb.get(strBytes);
          // Find null terminator
          int len = 0;
          while (len < 256 && strBytes[len] != 0) len++;
          String errorMessage = new String(strBytes, 0, len, StandardCharsets.UTF_8);

          return new QuestNavCommandResponse(commandId, success, errorMessage);
        }

        @Override
        public void pack(ByteBuffer bb, QuestNavCommandResponse value) {
          bb.putLong(value.commandId);
          bb.put((byte) (value.success ? 1 : 0));

          byte[] strBytes = value.errorMessage.getBytes(StandardCharsets.UTF_8);
          int len = Math.min(strBytes.length, 255);
          bb.put(strBytes, 0, len);
          // Fill remainder with 0
          for (int i = len; i < 256; i++) {
            bb.put((byte) 0);
          }
        }
      };
}
