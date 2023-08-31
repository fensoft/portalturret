//General
#include "Arduino.h"

//Network
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h> // Include the mDNS library
#include <ESP8266httpUpdate.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <AsyncElegantOTA.h>

//Storage
#include "LittleFS.h"

//Routines
#include <AceRoutine.h>

//Devices
#include <FastLED.h>
#include <Servo.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

void startWebServer();
void startWebSocket();

using namespace ace_routine;

//Tweak these according to servo speed
#define OPEN_DURATION 1000
#define CLOSE_STOP_DELAY 100
#define MAX_ROTATION 50

#define USE_AUDIO

#define BUSY D0
#define CENTER_LED D3
#define GUN_LEDS D4
#define RING_LEDS D8
#define SERVO_A D6
#define SERVO_B D7
#define WING_SWITCH D5
#define PID A0

#define CENTER_ANGLE 90
#define NUM_LEDS 8

#define GFORCE_PICKED_UP_MIN 8
#define GFORCE_PICKED_UP_MAX 12
#define GFORCE_STEADY_MIN 9.5
#define GFORCE_STEADY_MAX 10.5
#define TIPPED_OVER_Z_TRESHOLD 5

CRGB leds[NUM_LEDS];

#include "PortalTypes.h"

Servo wingServo;
Servo rotateServo;

SoftwareSerial mySoftwareSerial(RX, TX); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

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
bool fullyOpened;
bool alarm;
bool needsSetup;
bool myDFPlayerSetup = false;
bool shouldUpdate = false;
int diagnoseAction = -1;

bool isOpen() {
  return digitalRead(WING_SWITCH) == HIGH;
}

//For some reason we need to cache this value, as checking it every loop causes the webserver to freeze.
//https://github.com/me-no-dev/ESPAsyncWebServer/issues/944
bool isDetectingMotionCached = false;
unsigned long lastMotionCheck = 0;
bool isDetectingMotion() {
  unsigned long curMillis = millis();
  if (curMillis > lastMotionCheck + 50) {
    isDetectingMotionCached = analogRead(A0) > 512;
    lastMotionCheck = curMillis;
  }
  return isDetectingMotionCached;
}

bool isPlayingAudio() {
  return digitalRead(BUSY) == LOW;
}

#include "Accelerometer.h"
#include "Routines.h"
#include "StateBehaviour.h"

TurretMode currentTurretMode;
TurretState currentState = TurretState::Idle;
ManualState currentManualState = ManualState::Idle;

unsigned long detectTime = 0;
unsigned long undetectTime = 0;
unsigned long previousTime = 0;
unsigned long stateStartTime = 0;
unsigned long lastMovementTime = 0;

void setState(TurretState nextState) {

  //Stop the Wing Servos just in case;
  wingServo.write(STATIONARY_ANGLE);
  
  if (currentTurretMode == TurretMode::Automatic) {
    switch (nextState) {
      case TurretState::Activated:
        activatedRoutine.reset();
        break;
      case TurretState::Engaging:
        engagingRoutine.reset();
        break;
      case TurretState::Searching:
        searchingRoutine.reset();
        break;
      case TurretState::TargetLost:
        targetLostRoutine.reset();
        break;
      case TurretState::PickedUp:
        pickedUpRoutine.reset();
        break;
      case TurretState::Shutdown:
        shutdownRoutine.reset();
        break;
      case TurretState::ManualEngaging:
        manualEngagingRoutine.reset();
        break;
      case TurretState::Rebooting:
        rebootRoutine.reset();
        break;
    }
    stateStartTime = millis();
    currentState = nextState;
  }
}

void setManualState(ManualState nextState) {

  //Stop the Wing Servos just in case;
  wingServo.write(STATIONARY_ANGLE);

  
  if (currentTurretMode == TurretMode::Manual) {
    switch (nextState) {
      case ManualState::Opening:
        openWingsRoutine.reset();
        break;
      case ManualState::Closing:
        closeWingsRoutine.reset();
        break;
      case ManualState::Firing:
        manualEngagingRoutine.reset();
        break;
    }
    currentManualState = nextState;
  }
}

void manualRotation(unsigned long deltaTime) {
  manualMovementRoutine.runCoroutine();
}
void stateBehaviour() {

  unsigned long deltaTime = millis() - previousTime;
  previousTime = millis();

  if (currentTurretMode == TurretMode::Manual) {
    switch (currentManualState) {
      case ManualState::Idle:
        manualRotation(deltaTime);
        break;
      case ManualState::Opening:
        openWingsRoutine.runCoroutine();
        if (openWingsRoutine.isDone()) {
          setManualState(ManualState::Idle);
        }
        break;
      case ManualState::Closing:
        closeWingsRoutine.runCoroutine();
        if (closeWingsRoutine.isDone()) {
          setManualState(ManualState::Idle);
        }
        break;
      case ManualState::Firing:
        manualRotation(deltaTime);
        manualEngagingRoutine.runCoroutine();
        if (manualEngagingRoutine.isDone()) {
          setManualState(ManualState::Idle);
        }
        break;
    }
  }
  if (currentTurretMode == TurretMode::Automatic) {
    
    bool motionDetected = isDetectingMotion();
    float zMovement = (smoothZ / measurements * SENSORS_GRAVITY_STANDARD * ADXL345_MG2G_MULTIPLIER);
    bool pickedUp = accelerometerBuffered && (zMovement < GFORCE_PICKED_UP_MIN || zMovement > GFORCE_PICKED_UP_MAX);
    bool movedAround = accelerometerBuffered && (zMovement < GFORCE_STEADY_MIN || zMovement > GFORCE_STEADY_MAX);
    bool onItsSide = accelerometerBuffered && (zMovement < TIPPED_OVER_Z_TRESHOLD);
    
    if (movedAround) {
      lastMovementTime = millis();
    }

    if (pickedUp && currentState != TurretState::PickedUp && currentState != TurretState::Shutdown && currentState != TurretState::Rebooting) {
      setState(TurretState::PickedUp);
    }
    switch (currentState) {
      case TurretState::Idle:
        if (motionDetected) {
          setState(TurretState::Activated);
        }
        break;
      case TurretState::Activated:
        activatedRoutine.runCoroutine();
        if (activatedRoutine.isDone()) {
          if (motionDetected) {
            setState(TurretState::Engaging);
          } else {
            setState(TurretState::Searching);
          }
        }
        break;
      case TurretState::Searching:
        searchingRoutine.runCoroutine();
        if (millis() > stateStartTime + 10000) {
          setState(TurretState::TargetLost);
        }
        if (motionDetected && millis() > stateStartTime + 100) {
          setState(TurretState::Engaging);
        }
        break;
      case TurretState::TargetLost:
        targetLostRoutine.runCoroutine();
        if (targetLostRoutine.isDone()) {
          setState(TurretState::Idle);
        }
        break;
      case TurretState::Engaging:
        engagingRoutine.runCoroutine();
        if (engagingRoutine.isDone()) {
          if (wingsOpen) {
            setState(TurretState::Searching);
          } else {
            setState(TurretState::Idle);
          }
        }
        break;
      case TurretState::PickedUp:
        pickedUpRoutine.runCoroutine();
        if (onItsSide) {
          setState(TurretState::Shutdown);
        }
        else if (!movedAround && millis() > lastMovementTime + 5000) {
          setState(TurretState::Activated);
        }
        break;
      case TurretState::Shutdown:
        shutdownRoutine.runCoroutine();
        if (shutdownRoutine.isDone() && !onItsSide) {
          setState(TurretState::Rebooting);
        }
        break;
      case TurretState::Rebooting:
        rebootRoutine.runCoroutine();
        if (rebootRoutine.isDone()) {
          setState(TurretState::Idle);
        }
        break;
    }
  }
}

bool isConnected;

void UpdateLEDPreloader() {
  int t = floor(millis() / 10);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB((i + t) % 8 == 0 ? 255 : 0, 0, 0);
    FastLED.show();
  }
}

void setup()
{

  Serial.begin(9600);
  // Initialize LittleFS
  if (!LittleFS.begin()) {
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

  wingServo.attach(SERVO_A);
  rotateServo.attach(SERVO_B);

  rotateServo.write(90);
  delay(250);
  fullyOpened = false;
  wingServo.write(STATIONARY_ANGLE + 90);
  while(isOpen()) {
    delay(10);
  }
  delay(CLOSE_STOP_DELAY);
  wingServo.write(STATIONARY_ANGLE);

  pinMode(BUSY, INPUT);
  pinMode(WING_SWITCH, INPUT_PULLUP);

  File wifiCreds = LittleFS.open(WIFI_CRED_FILE, "r");
  String esid = wifiCreds.readStringUntil('\r'); wifiCreds.read();
  String epass = wifiCreds.readStringUntil('\r'); wifiCreds.read();

  WiFi.hostname("turret");
  WiFi.mode(WIFI_STA);
  WiFi.begin(esid, epass);

  unsigned long m = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Starting wifi");
    UpdateLEDPreloader();
    delay(50);
    if (m + 10000 < millis()) {
      WiFi.disconnect();
      break;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Not connected");
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);

    needsSetup = true;

    //Preemtive scan of networks, just in case.
    WiFi.scanNetworks(true);

    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP("Portal Turret");

  } else {
    Serial.println("Connected");
    Serial.println(WiFi.localIP());
    delay(500);
  }

  UpdateLEDPreloader();

  currentState = TurretState::Idle;
  currentManualState = ManualState::Idle;
  currentTurretMode = TurretMode::Automatic;

  wasOpen = isOpen();

  AsyncElegantOTA.begin(&server);

  UpdateLEDPreloader();

  startWebServer();
  startWebSocket();
  setupAccelerometer();

  UpdateLEDPreloader();

  Serial.println("Begin MDNS");
  if (MDNS.begin("turret", WiFi.localIP())) {
    Serial.println("MDNS setup");
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("http", "tcp", 81);
  } else {
    Serial.println("MDNS failed");
  }
  delay(200);
  Serial.end();

  UpdateLEDPreloader();

#ifdef USE_AUDIO
  mySoftwareSerial.begin(9600);
  delay(200);
  myDFPlayerSetup = myDFPlayer.begin(mySoftwareSerial);
  if (myDFPlayerSetup) myDFPlayer.volume(15);
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
}

void preloader(uint8_t led) {
  FastLED.clear();
  leds[led] = CRGB(255, 0, 0);
  FastLED.show();
}

void loop()
{
  MDNS.update();
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
        myDFPlayer.playFolder(1, random(1, 9));
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
    int a = analogRead(A0);

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
      (isPlayingAudio() ? 1 : 0),
    };
    webSocket.broadcastBIN(values, 12);
  }
}
