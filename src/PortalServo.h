#include <Arduino.h>
#include "config.h"

#ifdef ESP32
class Servo {
public:
  void attach(int pin, int channel, int freq, int resolution) {
    this->channel = channel;
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(pin, channel);
  }
  void write(int value) {
    int servoValue = value * (DUTYCYLEMAX - DUTYCYLEMIN) / 180 + DUTYCYLEMIN;
    ledcWrite(channel, servoValue);
  }
private:
  int channel;
};
#else
#include <Servo.h>
#endif