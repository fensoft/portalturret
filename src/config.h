#pragma once
//Tweak these according to servo speed
#define OPEN_DURATION 1000
#define CLOSE_STOP_DELAY 100
#define MAX_ROTATION 50

//#define USE_AUDIO

#ifdef ESP32
#define BUSY 5
#define CENTER_LED 18
#define GUN_LEDS 16
#define RING_LEDS 12
#define SERVO_A 9
#define SERVO_B 11
#define WING_SWITCH 7
#define PID 3
#define MP3_TX 40
#define MP3_RX 38
#else
#define BUSY D0
#define CENTER_LED D3
#define GUN_LEDS D4
#define RING_LEDS D8
#define SERVO_A D6
#define SERVO_B D7
#define WING_SWITCH D5
#define PID A0
#define MP3_TX TX
#define MP3_RX RX
#endif

#define CENTER_ANGLE 90
#define NUM_LEDS 8

#define GFORCE_PICKED_UP_MIN 8
#define GFORCE_PICKED_UP_MAX 12
#define GFORCE_STEADY_MIN 9.5
#define GFORCE_STEADY_MAX 10.5
#define TIPPED_OVER_Z_TRESHOLD 5

#define WIFI_CRED_FILE "/settings.txt"
#define STATIONARY_ANGLE 90