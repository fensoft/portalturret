#pragma once
#include <Arduino.h>

class LD2450 {
public:
  struct Packet {
    int16_t speed;
    int16_t x;
    int16_t y;
    uint16_t distance;
    bool valid;
  };

  // Constructor takes any Stream (Serial, SoftwareSerial, WiFiClient, etc.)
  LD2450(Stream &stream);

  // Call regularly in loop() to process incoming bytes
  Packet update();

private:
  static constexpr size_t MAX_PACKET = 256;
  static constexpr uint8_t TERM0 = 0x55;
  static constexpr uint8_t TERM1 = 0xCC;

  Stream &stream_;
  uint8_t buf_[MAX_PACKET];
  size_t len_ = 0;

  static uint16_t read_le16(const uint8_t *buf, size_t i);
  static int16_t pythonish_convert(uint16_t v);

  Packet parsePacket();
};
