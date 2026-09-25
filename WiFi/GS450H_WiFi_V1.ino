/*
The controller sends a framed telemetry record over serial at 19200 baud 8n1:
@v=630;i=12.4;p=7.8;m=1200;n=2400;o=42.1;r=44.0;q=50;...*

The first eight keys are the original Wi-Fi protocol:

v=pack voltage (0-700Volts)
i=current (0-1000Amps)
p=power (0-300kw)
m=mg1 rpm (0-10000rpm)
n=mg2 rpm (0-10000rpm)
o=mg1 temp (-20 to 120C)
r=mg2 temp (-20 to 120C)
q=oil-pump PWM command (0-100%, legacy field; not measured pressure)
*=end of string
xxx=three digit integer for each parameter eg p100 = 100kw.
The current v3 user firmware sends approximately once per second.

Older vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx* records are still accepted.
*/

// Import required libraries
#ifdef ESP32
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#else
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#define SPIFFS LittleFS
#endif
#include <Wire.h>
#include "secrets.h"

String v, i, p, m, n, o, r, q;
String telemetryJson = "{\"ok\":false}";
unsigned long lastFrameAt = 0;

const char* otaHostname = "GS450H-Inverter";
String serialFrame;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String PackVoltage() {
  return v;
}
String Current() {
  return i;
}
String Power() {
  return p;
}
String mg1RPM() {
  return m;
}
String mg2RPM() {
  return n;
}
String mg1Temp() {
  return o;
}
String mg2Temp() {
  return r;
}
String oilPumpPwm() {
  return q;
}

void setTelemetryField(const String& key, const String& value) {
  if (key == "v") v = value;
  else if (key == "i") i = value;
  else if (key == "p") p = value;
  else if (key == "m") m = value;
  else if (key == "n") n = value;
  else if (key == "o") o = value;
  else if (key == "r") r = value;
  else if (key == "q") q = value;
}

void addJsonField(String& json, const char* key, const String& value, bool numeric) {
  json += "\"";
  json += key;
  json += "\":";
  if (numeric && value.length() > 0) json += value;
  else {
    json += "\"";
    json += value;
    json += "\"";
  }
}

void parseFramedTelemetry(const String& frame) {
  if (frame.length() < 4 || frame.charAt(0) != '@' ||
      frame.charAt(frame.length() - 1) != '*') return;

  String fields[32];
  String values[32];
  uint8_t count = 0;
  int start = 1;
  while (start < frame.length() - 1 && count < 32) {
    int end = frame.indexOf(';', start);
    if (end < 0 || end > frame.length() - 1) end = frame.length() - 1;
    int equals = frame.indexOf('=', start);
    if (equals > start && equals < end) {
      fields[count] = frame.substring(start, equals);
      values[count] = frame.substring(equals + 1, end);
      setTelemetryField(fields[count], values[count]);
      count++;
    }
    start = end + 1;
  }

  if (count == 0) return;
  telemetryJson = "{\"ok\":true,\"ageMs\":0";
  for (uint8_t index = 0; index < count; index++) {
    bool numeric = fields[index] != "gear" && fields[index] != "gs";
    telemetryJson += ",";
    addJsonField(telemetryJson, fields[index].c_str(), values[index], numeric);
  }
  telemetryJson += "}";
  lastFrameAt = millis();
}

void parseLegacyTelemetry(const String& frame) {
  int comma[7];
  int start = 0;
  for (uint8_t index = 0; index < 7; index++) {
    comma[index] = frame.indexOf(',', start);
    if (comma[index] < 0) return;
    start = comma[index] + 1;
  }
  if (frame.charAt(0) != 'v' || frame.charAt(frame.length() - 1) != '*') return;
  v = frame.substring(1, comma[0]);
  i = frame.substring(comma[0] + 2, comma[1]);
  p = frame.substring(comma[1] + 2, comma[2]);
  m = frame.substring(comma[2] + 2, comma[3]);
  n = frame.substring(comma[3] + 2, comma[4]);
  o = frame.substring(comma[4] + 2, comma[5]);
  r = frame.substring(comma[5] + 2, comma[6]);
  q = frame.substring(comma[6] + 2, frame.length() - 1);
  telemetryJson = "{\"ok\":true,\"ageMs\":0";
  telemetryJson += ",";
  addJsonField(telemetryJson, "v", v, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "i", i, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "p", p, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "m", m, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "n", n, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "o", o, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "r", r, true);
  telemetryJson += ",";
  addJsonField(telemetryJson, "q", q, true);
  telemetryJson += "}";
  lastFrameAt = millis();
}

String getBGcolor() {
  File BGcolor = SPIFFS.open("/BGcolor.txt", "r");
  String value = BGcolor.readString();
  BGcolor.close();
  return (value);
}

String getHeading() {
  File Heading = SPIFFS.open("/Heading.txt", "r");
  String value = Heading.readString();
  Heading.close();
  return (value);
}

String getSsid() {
  File ssid = SPIFFS.open("/ssid.txt", "r");
  String value = ssid.readString();
  ssid.close();
  return (value);
}

String getPassword() {
  File password = SPIFFS.open("/password.txt", "r");
  String value = password.readString();
  password.close();
  return (value);
}

bool writeSetting(const char* path, const String& value) {
  File setting = SPIFFS.open(path, "w");
  if (!setting) {
    Serial.print("Unable to write ");
    Serial.println(path);
    return false;
  }
  setting.print(value);
  setting.close();
  return true;
}

String requestValue(AsyncWebServerRequest* request, const char* name) {
  if (request->hasParam(name, true)) return request->getParam(name, true)->value();
  if (request->hasParam(name)) return request->getParam(name)->value();
  return "";
}

bool requestHasValue(AsyncWebServerRequest* request, const char* name) {
  return request->hasParam(name, true) || request->hasParam(name);
}

String readConfigFile(const char* path) {
  File file = SPIFFS.open(path, "r");
  if (!file) return "";
  String value = file.readString();
  file.close();
  value.trim();
  return value;
}

void startNetwork() {
  String stationSsid = readConfigFile("/ssid.txt");
  String stationPassword = readConfigFile("/password.txt");
  WiFi.mode(WIFI_STA);
  if (stationSsid.length() > 0) {
    WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
    unsigned long started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 10000UL) {
      delay(100);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("GS450H-Inverter", WIFI_AP_PASSWORD);
    Serial.println("Wi-Fi STA unavailable; started fallback AP");
  } else {
    Serial.print("Wi-Fi STA connected: ");
    Serial.println(WiFi.localIP());
  }

  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(19200);

  // Initialize the deployed filesystem. ESP8266 uses LittleFS under the
  // existing SPIFFS name so route/file compatibility is preserved.
  if (!SPIFFS.begin()) {
    Serial.println("An error occurred while mounting the filesystem");
    return;
  }

  startNetwork();

  // Route for root / web pages
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html");
  });
  server.on("/admin", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/admin.html");
  });
  server.on("/highcharts.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts.js", "text/javascript");
  });
  server.on("/highcharts-more.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts-more.js", "text/javascript");
  });
  server.on("/solid-gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/solid-gauge.js", "text/javascript");
  });
  server.on("/PackVoltage", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", PackVoltage().c_str());
  });
  server.on("/Current", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", Current().c_str());
  });
    server.on("/Power", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", Power().c_str());
  });
  server.on("/mg1RPM", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg1RPM().c_str());
  });
  server.on("/mg2RPM", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg2RPM().c_str());
  });
  server.on("/mg1Temp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg1Temp().c_str());
  });
  server.on("/mg2Temp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg2Temp().c_str());
  });
  server.on("/oilPressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", oilPumpPwm().c_str());
  });
  server.on("/oilPumpPwm", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", oilPumpPwm().c_str());
  });
  server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest * request) {
    String response = telemetryJson;
    unsigned long age = lastFrameAt == 0 ? 4294967295UL : millis() - lastFrameAt;
    int marker = response.indexOf("\"ageMs\":0");
    if (marker >= 0) response.replace("\"ageMs\":0", "\"ageMs\":" + String(age));
    request->send(200, "application/json", response);
  });
  server.on("/getBGcolor", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getBGcolor().c_str());
  });
  server.on("/getSsid", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getSsid().c_str());
  });
  server.on("/getPassword", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getPassword().c_str());
  });
  server.on("/getStationSsid", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readConfigFile("/ssid.txt").c_str());
  });
  server.on("/getStationPassword", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readConfigFile("/password.txt").c_str());
  });
  server.on("/getHeading", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getHeading().c_str());
  });
  server.on("/setBGcolor", HTTP_ANY, [](AsyncWebServerRequest * request) {
    if (requestHasValue(request, "favcolor"))
      writeSetting("/BGcolor.txt", requestValue(request, "favcolor"));
    if (requestHasValue(request, "heading"))
      writeSetting("/Heading.txt", requestValue(request, "heading"));
    if (requestHasValue(request, "ssid"))
      writeSetting("/ssid.txt", requestValue(request, "ssid"));
    if (requestHasValue(request, "password"))
      writeSetting("/password.txt", requestValue(request, "password"));
    // The deployed layout uses these same files for station credentials.
    if (requestHasValue(request, "stationSsid"))
      writeSetting("/ssid.txt", requestValue(request, "stationSsid"));
    if (requestHasValue(request, "stationPassword"))
      writeSetting("/password.txt", requestValue(request, "stationPassword"));
    request->redirect("/");
  });

  server.begin();
}

void loop() {
  ArduinoOTA.handle();
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      serialFrame.trim();
      if (serialFrame.startsWith("@")) parseFramedTelemetry(serialFrame);
      else if (serialFrame.startsWith("v")) parseLegacyTelemetry(serialFrame);
      serialFrame = "";
    } else if (serialFrame.length() < 512) {
      serialFrame += incoming;
    } else {
      serialFrame = "";
    }
  }
}
