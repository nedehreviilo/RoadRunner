// RoadrunnerIO/src/main.cpp is copied from Mirza
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "controller.h"        // CONTROLLER_HTML, the page in PROGMEM

// =========================================================================
// Configuration: change these to match your hardware and calibration
// =========================================================================

// Wi-Fi access point
/* const char* AP_SSID     = "RC-Car-XX";          // Change name to something unique
const char* AP_PASSWORD = "maskinelement";
const int   AP_CHANNEL  = 1;                    // ESP-NOW will inherit this channel
 */
// Motor + servo pins (see the Electronics and Wiring page)
const int RPWM_PIN      = 16;
const int LPWM_PIN      = 17;
const int SERVO_PIN     = 13;
const int MOTOR_EN_PIN  = 19;                   // R_EN + L_EN tied together to this GPIO (NOT 12, strapping pin)
const int BATT_PIN      = 34;                   // 3S divider midpoint, ADC1, input-only

// Optional BTS7960 current sense: set to -1 to disable until you wire R_IS / L_IS.
// When wired, each pin outputs ~ I_motor / 8500 amps to GND through a sense resistor.
const int R_IS_PIN      = -1;                   // e.g. 32 once wired
const int L_IS_PIN      = -1;                   // e.g. 33 once wired

// LEDC channels: keep distinct from the ESP32Servo library (channels 0–3)
const int RPWM_CHANNEL  = 4;
const int LPWM_CHANNEL  = 5;
const int SERVO_CHANNEL = 0;
const int MOTOR_FREQ    = 20000;                // 20 kHz, above audible
const int MOTOR_RES     = 8;                    // 0..255 duty
const int SERVO_FREQ    = 50;
const int SERVO_RES     = 16;                   // 65 536 ticks per 20 ms

// Servo calibration: replace with YOUR measured end stops
// Default values should be safe
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2480;

// 3S battery calibration: replace with YOUR two-point readings
const long ADC_AT_9V   = 1880;                  // raw ADC at 9.0 V
const long ADC_AT_12V6 = 2630;                  // raw ADC at 12.6 V
const long LOW_BATTERY_WARN_mV = 9600;          // 3S cutoff (~3.2 V/cell)

// Drivetrain: used to convert motor RPM into vehicle km/h
const float WHEEL_DIAM_M  = 0.080f;             // wheel diameter in meters, measure yours
const float GEAR_RATIO    = 1.0f;               // motor turns per wheel turn (1.0 = direct drive)

// Command-source arbitration windows
const unsigned long ESPNOW_PRIO_MS = 200;       // joystick wins if last packet < 200 ms ago
const unsigned long FAILSAFE_MS    = 500;       // both stale > 500 ms → motor off

// =========================================================================
// State: written by callbacks, read by loop()
// =========================================================================

typedef struct {
  int16_t steering;   // -255 .. +255
  int16_t throttle;   // -255 .. +255
  bool    button;
  int16_t batt_mv;    // XIAO 1S battery, mV, filled in by the optional remote
} ControlData;

ControlData espnowCmd = {0, 0, false, 0};       // last ESP-NOW packet
ControlData webCmd    = {0, 0, false, 0};       // last WebSocket message
ControlData activeCmd = {0, 0, false, 0};       // whichever is currently driving

unsigned long lastEspNowMs = 0;
unsigned long lastWebMs    = 0;
const char*   activeSource = "failsafe";

bool motorsEnabled = false;                     // off at boot, user has to enable from the UI

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =========================================================================
// Battery monitor: two-point linear interpolation
// =========================================================================

long readBatteryMillivolts() {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(BATT_PIN);
  int raw = sum / 16;
  return map(raw, ADC_AT_9V, ADC_AT_12V6, 9000, 12600);
}

// =========================================================================
// Motor + servo: drive the BTS7960 and the steering servo
// =========================================================================

uint32_t usToServoDuty(int us) {
  return (uint32_t)((uint64_t)us * 65536ULL * SERVO_FREQ / 1000000ULL);
}

void stopMotor() {
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
}

void motorSetup() {
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);              // motors hardware-disabled at boot

  ledcSetup(RPWM_CHANNEL, MOTOR_FREQ, MOTOR_RES);
  ledcSetup(LPWM_CHANNEL, MOTOR_FREQ, MOTOR_RES);
  ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);
  ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
}

void setMotorEnable(bool on) {
  motorsEnabled = on;
  digitalWrite(MOTOR_EN_PIN, on ? HIGH : LOW);  // BTS7960 R_EN + L_EN
  if (!on) stopMotor();                         // belt-and-braces: also kill PWM
}

void servoSetup() {
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  int centerUs = (SERVO_MIN_US + SERVO_MAX_US) / 2;
  ledcWrite(SERVO_CHANNEL, usToServoDuty(centerUs));
}
/* 
void applyControl(const ControlData& cmd) {
  // Throttle: split sign across RPWM and LPWM
  int t = constrain(cmd.throttle, -255, 255);
  if (t > 0) {
    ledcWrite(RPWM_CHANNEL, t);
    ledcWrite(LPWM_CHANNEL, 0);
  } else {
    ledcWrite(RPWM_CHANNEL, 0);
    ledcWrite(LPWM_CHANNEL, -t);
  }
  // Steering: map -255..+255 to calibrated pulse width
  int us = map(constrain(cmd.steering, -255, 255), -255, 255,
               SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(SERVO_CHANNEL, usToServoDuty(us));
}
 */
// =========================================================================
// Command-source arbitration: joystick > web > failsafe
// =========================================================================
/* 
void selectActiveCommand() {
  unsigned long now = millis();
  if (now - lastEspNowMs < ESPNOW_PRIO_MS) {
    activeCmd    = espnowCmd;
    activeSource = "joystick";
  } else if (now - lastWebMs < FAILSAFE_MS) {
    activeCmd    = webCmd;
    activeSource = "web";
  } else {
    activeCmd    = {0, 0, false};
    activeSource = "failsafe";
    stopMotor();
    return;                                     // skip applyControl when failed-safe
  }
  applyControl(activeCmd);
}
 */
// =========================================================================
// ESP-NOW receive callback: fires whenever a packet from the XIAO arrives
// =========================================================================
/* 
void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(ControlData)) return;       // ignore unexpected lengths
  memcpy(&espnowCmd, data, sizeof(espnowCmd));
  lastEspNowMs = millis();
}

void espNowSetup() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowReceive);
}
 */
// =========================================================================
// WebSocket: short text frames from the phone:
//   "steering,throttle"     = joystick state (-255 .. +255 each)
//   "en1" / "en0"           = motors-enabled toggle
// =========================================================================
/* 
void parseWebMessage(uint8_t* data, size_t len) {
  data[len] = 0;                                // null-terminate so String works
  String msg = (char*)data;

  if (msg.startsWith("en")) {
    setMotorEnable(msg.length() > 2 && msg[2] == '1');
    return;
  }

  int comma = msg.indexOf(',');
  if (comma <= 0) return;
  webCmd.steering = msg.substring(0, comma).toInt();
  webCmd.throttle = msg.substring(comma + 1).toInt();
  webCmd.button   = false;
  lastWebMs = millis();
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_DATA) parseWebMessage(data, len);
}
 */
// =========================================================================
// HTTP: serve the embedded controller page from PROGMEM
// =========================================================================
/* 
void httpSetup() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", CONTROLLER_HTML);
  });
  server.begin();
}
 */
// =========================================================================
// Motor RPM, vehicle speed, BTS7960 current: stubs for now
// =========================================================================

// Motor shaft RPM. Returns 0 until you wire a magnetic encoder.
float readMotorRpm() {
  // TODO: read encoder angle, differentiate over time, return shaft RPM.
  return 0.0f;
}

// Convert motor RPM to vehicle speed in km/h, using the drivetrain constants
// at the top of the file. With the default GEAR_RATIO = 1.0 this assumes
// direct drive, set the real ratio once you build a gearbox.
float rpmToKmh(float motorRpm) {
  float wheelRpm = motorRpm / GEAR_RATIO;
  float circ_m   = WHEEL_DIAM_M * 3.14159265f;
  return wheelRpm * circ_m * 60.0f / 1000.0f;   // m/min → km/h
}

// BTS7960 current sense: returns 0 unless you wire R_IS / L_IS to ADC pins.
// The chip pulls roughly I_motor / 8500 amps out of each IS pin; with a 1 kΩ
// sense resistor to GND that's ~0.118 mV per ampere of motor current.
// Tune the gain once you have actual hardware.
float readCurrent(int pin) {
  if (pin < 0) return 0.0f;
  int raw = analogRead(pin);
  float v = raw * (3.3f / 4095.0f);
  // I_motor = V_sense / R_sense * 8500.   With R_sense = 1 kΩ:
  return v / 1000.0f * 8500.0f;
}

// =========================================================================
// Telemetry: pushed to all connected web clients every 500 ms
// =========================================================================
/* 
void sendTelemetry() {
  long batt3s_mv = readBatteryMillivolts();
  long batt1s_mv = espnowCmd.batt_mv;            // sent by the XIAO transmitter
  bool low3s    = batt3s_mv < LOW_BATTERY_WARN_mV;
  float rpm     = readMotorRpm();
  float speed   = rpmToKmh(rpm);
  float ir      = readCurrent(R_IS_PIN);
  float il      = readCurrent(L_IS_PIN);

  String json = "{";
  json += "\"batt3s_v\":"   + String(batt3s_mv / 1000.0, 2);
  json += ",\"batt1s_v\":"  + String(batt1s_mv / 1000.0, 2);
  json += ",\"batt_low\":"  + String(low3s ? "true" : "false");
  json += ",\"motor_rpm\":" + String(rpm, 0);
  json += ",\"speed_kmh\":" + String(speed, 1);
  json += ",\"current_r\":" + String(ir, 2);
  json += ",\"current_l\":" + String(il, 2);
  json += ",\"enabled\":"   + String(motorsEnabled ? "true" : "false");
  json += ",\"source\":\""  + String(activeSource) + "\"";
  json += "}";
  ws.textAll(json);

  if (low3s) Serial.println("!! LOW 3S BATTERY (< 9.6 V) !!");
}
 */
// =========================================================================
// Wi-Fi AP: must come up BEFORE espNowSetup() so they share a channel
// =========================================================================
/* 
void wifiSetup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.printf("AP SSID: %s\n", AP_SSID);
  Serial.print("AP IP: ");      Serial.println(WiFi.softAPIP());
  Serial.printf("Channel: %d\n", AP_CHANNEL);
}
 */
// =========================================================================
// Arduino entry points
// =========================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10); // wait for USB-CDC enumeration

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  motorSetup();
  servoSetup();
  wifiSetup();        // start AP first, locks the radio to AP_CHANNEL
  espNowSetup();      // ESP-NOW now inherits the AP's channel
  httpSetup();        // serves controller.h from PROGMEM
}

void loop() {
  selectActiveCommand();              // arbitrate every loop iteration
  ws.cleanupClients();

  static unsigned long lastTelem = 0;
  if (millis() - lastTelem > 500) {
    sendTelemetry();
    lastTelem = millis();
  }
  delay(20);
}