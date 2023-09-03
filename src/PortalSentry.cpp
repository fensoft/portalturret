//General
#include <Arduino.h>

//Network
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h> // Include the mDNS library
#include <ESP8266httpUpdate.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <AsyncElegantOTA.h>

//Storage
#include "LittleFS.h"

//Devices
#include <FastLED.h>
#include "config.h"
#include "PortalServo.h"
#include "SoftwareSerial.h"

#include "StateMachine.hpp"

void startWebServer();
void startWebSocket();

CRGB leds[NUM_LEDS];

#include "PortalTypes.h"

Servo wingServo;
Servo rotateServo;

SoftwareSerial mySoftwareSerial(MP3_RX, MP3_TX); // RX, TX
#include "mp3_driver.h"
#include "mp3_driver_factory.h"
Mp3Driver* mp3_driver;

#define FREQ 50     //one clock is 20 ms
#define FREQ_MINIMUM 205 //1ms is 1/20, of 4096
#define FREQ_MAXIMUM 410 //2ms is 2/20, of 4096
int currentMoveSpeed = 0;

AsyncWebServer server = AsyncWebServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);

bool websocketStarted;
bool diagnoseMode = false;
unsigned long nextWebSocketUpdateTime = 0;

bool wingsOpen;
bool wasOpen;
bool needsSetup;
bool shouldUpdate = false;
int diagnoseAction = -1;

#include "Accelerometer.h"

TurretMode currentTurretMode;
TurretState currentState = TurretState::Idle;

unsigned long detectTime = 0;
unsigned long undetectTime = 0;
unsigned long previousTime = 0;
bool isConnected;

#ifdef ESP32
#include <DNSServer.h>
DNSServer dnsServer;
#endif

void UpdateLEDPreloader() {
  int t = floor(millis() / 100);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB((i + t) % 8 == 0 ? 255 : 0, 0, 0);
    FastLED.show();
  }
}

void setup()
{
  Serial.begin(9600);
  LOG_INIT(&Serial);
  mySoftwareSerial.begin(9600);
  delay(500);
#ifdef USE_AUDIO
  Serial.println("Starting MP3 module...");
  mp3_driver = new_mp3_driver(&mySoftwareSerial, BUSY);
  mp3_driver->setVolume(15);
  mp3_driver->playSongFromFolder(1, 1+random(mp3_driver->getFileCountInFolder(1)));
#endif

  Serial.println("LittleFS initializing...");
  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Error while initializing littleFS");
    while (true) { }
    return;
  }
  Serial.println("LittleFS Initialized");

  FastLED.addLeds<WS2812, RING_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(84);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
    FastLED.show();
  }

  pinMode(GUN_LEDS, OUTPUT);
#ifdef ESP32
  wingServo.attach(SERVO_A, 1, 50, 13);
  rotateServo.attach(SERVO_B, 2, 50, 13);
#else
  wingServo.attach(SERVO_A);
  rotateServo.attach(SERVO_B);
#endif

  rotateServo.write(90);
  delay(250);
  wingServo.write(STATIONARY_ANGLE + 90);
  while(isOpen()) {
    delay(10);
  }
  delay(CLOSE_STOP_DELAY);
  wingServo.write(STATIONARY_ANGLE);

  pinMode(WING_SWITCH, INPUT_PULLUP);

  File wifiCreds = LittleFS.open(WIFI_CRED_FILE, "r");
  String esid = wifiCreds.readStringUntil('\r'); wifiCreds.read();
  String epass = wifiCreds.readStringUntil('\r'); wifiCreds.read();

  WiFi.hostname("turret");
  WiFi.mode(WIFI_STA);
  WiFi.begin(esid.c_str(), epass.c_str());
  Serial.println("Connecting to " + esid + "(" + epass + ")");

  unsigned long m = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("\r");
    Serial.print(millis());
    UpdateLEDPreloader();
    delay(20);
    if (m + 10000 < millis()) {
      WiFi.disconnect();
      break;
    }
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Not connected. AP MODE");
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);

    needsSetup = true;

    //Preemtive scan of networks, just in case.
    WiFi.scanNetworks(true);

    IPAddress local_IP(8, 8, 8, 8);
    IPAddress gateway(8, 8, 8, 8);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP("Portal Turret");

#ifdef ESP32
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError); 
    dnsServer.start(53, "*", local_IP);
#endif

  } else {
    Serial.println("Connected");
    Serial.println(WiFi.localIP());
    delay(500);
  }

  UpdateLEDPreloader();

  currentState = TurretState::Idle;
  currentTurretMode = TurretMode::Automatic;

  wasOpen = isOpen();

  AsyncElegantOTA.begin(&server);

  UpdateLEDPreloader();

  startWebServer();
  startWebSocket();
  setupAccelerometer();

  UpdateLEDPreloader();

  Serial.println("Begin MDNS");
#ifndef ESP32
  if (MDNS.begin("turret", WiFi.localIP())) {
#else
  if (MDNS.begin("turret")) {
#endif
    Serial.println("MDNS setup");
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("http", "tcp", 81);
  } else {
    Serial.println("MDNS failed");
  }
  delay(200);
#ifndef ESP32
  Serial.end();
#endif

  UpdateLEDPreloader();

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      //Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      //Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      //Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      //Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      //Serial.println("End Failed");
    }
  });
  ArduinoOTA.setHostname("turret");
  ArduinoOTA.begin();

  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(255, 0, 0);
    FastLED.show();
  }

  previousTime = millis();
  Serial.println("Setup complete");
}

void preloader(uint8_t led) {
  FastLED.clear();
  leds[led] = CRGB(255, 0, 0);
  FastLED.show();
}

void loop()
{
#ifndef ESP32
  MDNS.update();
#else
  dnsServer.processNextRequest();
#endif
  //if (!isConnected) return;

  wingsOpen = isOpen();
  ArduinoOTA.handle();
  webSocket.loop();
  if (!diagnoseMode) {
    stateBehaviour();
  } else {
    switch (diagnoseAction) {
      case 0:
        wingServo.write(STATIONARY_ANGLE - 90);
        delay(250);
        wingServo.write(STATIONARY_ANGLE);
        break;
      case 1:
        wingServo.write(STATIONARY_ANGLE + 90);
        delay(250);
        wingServo.write(STATIONARY_ANGLE);
        break;
      case 2:
        rotateServo.write(50);
        delay(1000);
        rotateServo.write(90);
        break;
      case 3:
        rotateServo.write(130);
        delay(1000);
        rotateServo.write(90);
        break;
      case 4:
        analogWrite(GUN_LEDS, 255);
        delay(1000);
        analogWrite(GUN_LEDS, 0);
        break;
      case 5:
        fill_solid(leds, NUM_LEDS, CRGB::Red);
        FastLED.show();
        delay(1000);
        fill_solid(leds, NUM_LEDS, CRGB::Green);
        FastLED.show();
        delay(1000);
        fill_solid(leds, NUM_LEDS, CRGB::Blue);
        FastLED.show();
        delay(1000);
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        break;
      case 6:
#ifdef USE_AUDIO
        mp3_driver->playSongFromFolder(1, random(1, 9));
#endif
        break;
    }
    diagnoseAction = -1;
  }

  if (currentMoveSpeed > 0 && wasOpen && !wingsOpen) {
    currentMoveSpeed = 0;
    delay(CLOSE_STOP_DELAY);
    wingServo.write(STATIONARY_ANGLE);
  }

  wasOpen = wingsOpen;

  if (websocketStarted && millis() > nextWebSocketUpdateTime)
  {
    nextWebSocketUpdateTime = millis() + 30;
    int a = analogRead(PID);

    updateAccelerometer();

    int16_t x = smoothX / measurements;
    int16_t y = smoothY / measurements;
    int16_t z = smoothZ / measurements;

    uint8_t values[] = {
      (x >> 8),
      (x & 0xFF),
      (y >> 8),
      (y & 0xFF),
      (z >> 8),
      (z & 0xFF),
      (!isOpen() ? 1 : 0),
      (isDetectingMotion() ? 1 : 0),
      ((uint8_t)(a >> 8)) & 0xFF,
      ((uint8_t)a) & 0xFF,
      (uint8_t) currentState,
      (mp3_driver->isBusy() ? 1 : 0),
    };
    webSocket.broadcastBIN(values, 12);
  }
}
