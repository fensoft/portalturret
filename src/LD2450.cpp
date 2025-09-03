#include "LD2450.h"

LD2450::LD2450(Stream &stream)
  : stream_(stream) {}

LD2450::Packet LD2450::update() {
  Packet pkt = {0, 0, 0, false};

  while (stream_.available()) {
    uint8_t b = (uint8_t)stream_.read();
    if (len_ < MAX_PACKET) {
      buf_[len_++] = b;
    } else {
      len_ = 0; // reset on overflow
    }

    // Check for terminator
    if (len_ >= 2 && buf_[len_ - 2] == TERM0 && buf_[len_ - 1] == TERM1) {
      pkt = parsePacket();
      len_ = 0; // reset for next packet
      break;    // return immediately after one packet
    }
  }
  return pkt;
}

uint16_t LD2450::read_le16(const uint8_t *buf, size_t i) {
  return (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8);
}

int16_t LD2450::pythonish_convert(uint16_t v) {
  if (v > 32768) return (int16_t)(32768 - (int32_t)v);
  return (int16_t)v;
}

LD2450::Packet LD2450::parsePacket() {
  Packet pkt = {0, 0, 0, false};
  if (len_ < 10) return pkt;

  uint16_t rawX = read_le16(buf_, 4);
  uint16_t rawY = read_le16(buf_, 6);
  uint16_t rawS = read_le16(buf_, 8);
  uint16_t rawD = read_le16(buf_, 10);

  pkt.x = pythonish_convert(rawX);
  pkt.y = pythonish_convert(rawY);
  pkt.speed = pythonish_convert(rawS);
  pkt.distance = rawD;
  pkt.valid = true;

  return pkt;
}
