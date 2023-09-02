#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <PortalTypes.h>
#include <LittleFS.h>
#ifdef ESP32
#include <Deneyap_Servo.h>
#else
#include <Servo.h>
#endif
#include "config.h"

extern AsyncWebServer server;
extern TurretMode currentTurretMode;
extern Servo rotateServo;
extern Servo wingServo;
extern int currentRotateAngle;
extern bool diagnoseMode;
extern int diagnoseAction;
extern int currentMoveSpeed;
void setManualState(ManualState nextState);
String processor(const String &var);
void requestReboot();
void setState(TurretState nextState);

class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request) override {
    return request->host() != WiFi.softAPIP().toString();
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->redirect(String() + "http://" +  WiFi.softAPIP().toString() + "/setup");
  }
};

void startWebServer()
{
  Serial.println("Start webserver");
  server.serveStatic("/", LittleFS, "/");
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(LittleFS, "index.html", String(), false, processor);
  });

  server.onNotFound([](AsyncWebServerRequest * request)
  {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404);
    }
  });

  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("Do Setup");
    request->send(LittleFS, "/setup.html", String(), false, processor);
  });

  server.on("/setup", HTTP_POST, [](AsyncWebServerRequest * request) {

    if (request->hasParam("ssid", true) && request->hasParam("pw", true)) {
      AsyncWebParameter* ssid = request->getParam("ssid", true);
      AsyncWebParameter* pw = request->getParam("pw", true);

      Serial.println("Saving SSID " + ssid->value() + "(" + pw->value() + ")");

      File wifiCreds = LittleFS.open(WIFI_CRED_FILE, FILE_WRITE);
      wifiCreds.print(ssid->value());
      wifiCreds.print("\r\n");
      wifiCreds.print(pw->value());
      wifiCreds.print("\r\n");
      wifiCreds.flush();
      wifiCreds.close();
    }

    request->send(200, "text/html", "ok");
    LittleFS.end();
    requestReboot();
  });

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest * request) {
    String json = "[";
    int n = WiFi.scanComplete();
    if (n == -2) {
      WiFi.scanNetworks(true);
    } else if (n) {
      for (int i = 0; i < n; ++i) {
        if (i) json += ",";
        json += "{";
        json += "\"rssi\":" + String(WiFi.RSSI(i));
        json += ",\"ssid\":\"" + WiFi.SSID(i) + "\"";
        json += ",\"bssid\":\"" + WiFi.BSSIDstr(i) + "\"";
        json += ",\"channel\":" + String(WiFi.channel(i));
        json += ",\"secure\":" + String(WiFi.encryptionType(i));
#ifndef ESP32
        json += ",\"hidden\":" + String(WiFi.isHidden(i) ? "true" : "false");
#endif
        json += "}";
      }
      WiFi.scanDelete();
      if (WiFi.scanComplete() == -2) {
        WiFi.scanNetworks(true);
      }
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  server.on("/set_mode", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    if (request->hasParam("mode", true))
    {
      AsyncWebParameter *modeParam = request->getParam("mode", true);
      currentTurretMode = (TurretMode) modeParam->value().toInt();
      currentRotateAngle = 90;
      request->send(200, "text/html", "State set");
    } else {
      request->send(200, "text/html", "Failed to set mode");
    }
  });

  server.on("/set_state", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    if (request->hasParam("state", true))
    {
      AsyncWebParameter *stateParam = request->getParam("state", true);
      int state = stateParam->value().toInt();
      setState((TurretState) state);
      request->send(200, "text/html", "State set");
    }
    else
    {
      request->send(200, "text/html", "No state sent");
    }
  });

  server.on("/diagnose", HTTP_GET, [](AsyncWebServerRequest * request) {
    diagnoseMode = true;
    request->send(LittleFS, "/diagnose.html", String(), false, processor);
  });

  server.on("/diagnose", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    if (request->hasParam("action", true))
    {
      AsyncWebParameter *stateParam = request->getParam("action", true);
      diagnoseAction = stateParam->value().toInt();
      request->send(200, "text/html", "Diagnose");
    }
    else
    {
      request->send(200, "text/html", "No Action Sent");
    }
  });

  server.on("/set_open", HTTP_POST, [](AsyncWebServerRequest * request) {
    if (currentTurretMode == TurretMode::Manual) {
      if (request->hasParam("open", true))
      {
        AsyncWebParameter *openParam = request->getParam("open", true);
        if (openParam->value().toInt() == 1) {
          setManualState(ManualState::Opening);
          request->send(200, "text/html", "Opening");
        } else {
          setManualState(ManualState::Closing);
          request->send(200, "text/html", "Closing");
        }
      } else {
        request->send(200, "text/html", "No state sent");
      }
    } else {
      request->send(200, "text/html", "Not in Manual mode");
    }
  });

  server.on("/set_angle", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    //Serial.println("Set angle request");
    if (request->hasParam("angle", true))
    {
      AsyncWebParameter *angleParam = request->getParam("angle", true);
      AsyncWebParameter *servoParam = request->getParam("servo", true);
      int angle = angleParam->value().toInt();
      int servo = servoParam->value().toInt();
      currentMoveSpeed = angle;
      if (servo == 0) {
        wingServo.write(STATIONARY_ANGLE + angle);
      } else {
        rotateServo.write(90 + angle);
      }

      request->send(200, "text/html", "Angle set");
    }
    else
    {
      request->send(200, "text/html", "No angle sent");
    }
  });

  server.on("/reset_wifi", HTTP_GET, [](AsyncWebServerRequest * request) {
    WiFi.disconnect();
    request->send(200, "text/html", "Wifi reset");
  });

  //DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
}

void requestReboot() {
  while (true) {
    int i = 0;
  }
}

String processor(const String &var)
{
  if (var == "IP")
    return WiFi.localIP().toString();
  return String();
}
