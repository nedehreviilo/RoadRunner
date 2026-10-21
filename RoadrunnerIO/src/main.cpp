#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <math.h>              // >>> RAMP DOWN: Krävs för fabsf() <<<
#include "controller.h"        // CONTROLLER_HTML, the page in PROGMEM
 
// =========================================================================
// Configuration: change these to match your hardware and calibration
// =========================================================================
 
// Wi-Fi access point
const char* AP_SSID     = "Chaos-Car";          // Change name to something unique
const char* AP_PASSWORD = "helloworld";
const int   AP_CHANNEL  = 1;                    // ESP-NOW will inherit this channel
 
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
const int SERVO_MIN_US = 250;
const int SERVO_MAX_US = 1100;
 
// 3S battery calibration: replace with YOUR two-point readings
const long ADC_AT_9V   = 2445;                  // raw ADC at 9.0 V
const long ADC_AT_12V6 = 3710;                  // raw ADC at 12.6 V
const long LOW_BATTERY_WARN_mV = 9600;          // 3S cutoff (~3.2 V/cell)
 
// Drivetrain: used to convert motor RPM into vehicle km/h
const float WHEEL_DIAM_M  = 0.100f;             // wheel diameter in meters, measure yours
const float GEAR_RATIO    = 1.0f;               // motor turns per wheel turn (1.0 = direct drive)
 
// Command-source arbitration windows
const unsigned long ESPNOW_PRIO_MS = 200;       // joystick wins if last packet < 200 ms ago
const unsigned long FAILSAFE_MS    = 500;       // both stale > 500 ms → motor off
 
// >>> MJUKSTART & FAILSAFE START: Tidsinställningar för acceleration och inbromsning <<<
const unsigned long DEFAULT_ACCEL_RAMP_MS = 800; // Tid i ms från 0 till full gas (255)
const unsigned long DEFAULT_DECEL_RAMP_MS = 400; // Tid i ms från full gas (255) till stopp (0)
volatile unsigned long accelRampMs = DEFAULT_ACCEL_RAMP_MS;
volatile unsigned long decelRampMs = DEFAULT_DECEL_RAMP_MS;
// >>> MJUKSTART & FAILSAFE SLUT <<<
 
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
 
float         currentThrottle = 0.0f;           // Faktisk utgående gas (-255.0 .. +255.0)
unsigned long lastRampMs      = 0;              // Tidsstämpel för föregående loop-varv
 
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
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
  currentThrottle = 0.0f;
}
 
void motorSetup() {
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);              // motors hardware-disabled at boot
 
  ledcAttach(RPWM_PIN, MOTOR_FREQ, MOTOR_RES);
  ledcAttach(LPWM_PIN, MOTOR_FREQ, MOTOR_RES);
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
}
 
void setMotorEnable(bool on) {
  motorsEnabled = on;
  digitalWrite(MOTOR_EN_PIN, on ? HIGH : LOW);  // BTS7960 R_EN + L_EN
  if (!on) stopMotor();                         // belt-and-braces: also kill PWM
}
 
void servoSetup() {
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  int centerUs = (SERVO_MIN_US + SERVO_MAX_US) / 2;
  ledcWrite(SERVO_PIN, usToServoDuty(centerUs));
}
 
// >>> MJUKSTART & FAILSAFE START: Matematiska hjälpfunktioner <<<
 
static float stepPerMs(unsigned long rampMs) {
  if (rampMs == 0) return 1.0e6f;
  return 255.0f / rampMs;
}
 
static bool isSlowingDown(float cur, float tgt) {
  bool reversing = (cur > 0.0f && tgt < 0.0f) || (cur < 0.0f && tgt > 0.0f);
  return reversing || fabsf(tgt) < fabsf(cur);
}
 
static float approach(float cur, float goal, float maxStep) {
  if (cur < goal) return min(cur + maxStep, goal);
  return max(cur - maxStep, goal);
}
 
static float decelToward(float cur, float tgt, float dtMs) {
  bool reversing = (cur > 0.0f && tgt < 0.0f) || (cur < 0.0f && tgt > 0.0f);
  float goal = reversing ? 0.0f : tgt;
  return approach(cur, goal, stepPerMs(decelRampMs) * dtMs);
}
 
// Denna funktion hanterar både upp-ramp och ner-ramp beroende på vad föraren gör
static float manualRamp(float cur, float tgt, float dtMs) {
  if (isSlowingDown(cur, tgt)) return decelToward(cur, tgt, dtMs);
  return approach(cur, tgt, stepPerMs(accelRampMs) * dtMs);
}
// >>> MJUKSTART & FAILSAFE SLUT <<<
 
void applyControl(const ControlData& cmd) {
  unsigned long now = millis();
  float dtMs = (lastRampMs == 0) ? 0.0f : (float)(now - lastRampMs);
  lastRampMs = now;
 
  if (!motorsEnabled) {
    currentThrottle = 0.0f;
  } else {
    float targetThrottle = (float)constrain(cmd.throttle, -255, 255);
    // >>> MJUKSTART & FAILSAFE START: Applicerar manuell ramp för alla rörelser <<<
    currentThrottle = manualRamp(currentThrottle, targetThrottle, dtMs);
    // >>> MJUKSTART & FAILSAFE SLUT <<<
  }
 
  int t = constrain((int)roundf(currentThrottle), -255, 255);
 
  if (t > 0) {
    ledcWrite(RPWM_PIN, t);
    ledcWrite(LPWM_PIN, 0);
  } else {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, -t);
  }
 
  int us = map(constrain(cmd.steering, -255, 255), -255, 255,
               SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(SERVO_PIN, usToServoDuty(us));
}
 
// =========================================================================
// Command-source arbitration: joystick > web > failsafe
// =========================================================================
 
void selectActiveCommand() {
  unsigned long now = millis();
  if (now - lastEspNowMs < ESPNOW_PRIO_MS) {
    activeCmd    = espnowCmd;
    activeSource = "joystick";
  } else if (now - lastWebMs < FAILSAFE_MS) {
    activeCmd    = webCmd;
    activeSource = "web";
  } else {
    activeCmd    = {0, 0, false, 0};
    activeSource = "failsafe";
    // >>> MJUKSTART & FAILSAFE START: Failsafe <<<
    // activeCmd.throttle sätts till 0 ovan. applyControl(activeCmd) kommer
    // nu rampa ner till 0 mjukt över 400ms (decelRampMs) om anslutningen bryts.
    // >>> MJUKSTART & FAILSAFE SLUT <<<
  }
  applyControl(activeCmd);
}
 
// =========================================================================
// ESP-NOW receive callback: fires whenever a packet from the XIAO arrives
// =========================================================================
 
void onEspNowReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ControlData)) return;
  memcpy(&espnowCmd, incomingData, sizeof(espnowCmd));
  lastEspNowMs = millis();
}
 
void espNowSetup() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowReceive);
}
 
// =========================================================================
// WebSocket: short text frames from the phone
// =========================================================================
 
void parseWebMessage(uint8_t* data, size_t len) {
  data[len] = 0;
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
 
// =========================================================================
// HTTP: serve the embedded controller page from PROGMEM
// =========================================================================
 
void httpSetup() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", CONTROLLER_HTML);
  });
  server.begin();
}
 
// =========================================================================
// Motor RPM, vehicle speed, BTS7960 current: stubs for now
// =========================================================================
 
float readMotorRpm() {
  return 0.0f;
}
 
float rpmToKmh(float motorRpm) {
  float wheelRpm = motorRpm / GEAR_RATIO;
  float circ_m   = WHEEL_DIAM_M * 3.14159265f;
  return wheelRpm * circ_m * 60.0f / 1000.0f;
}
 
float readCurrent(int pin) {
  if (pin < 0) return 0.0f;
  int raw = analogRead(pin);
  float v = raw * (3.3f / 4095.0f);
  return v / 1000.0f * 8500.0f;
}
 
// =========================================================================
// Telemetry: pushed to all connected web clients every 500 ms
// =========================================================================
 
void sendTelemetry() {
  long batt3s_mv = readBatteryMillivolts();
  long batt1s_mv = espnowCmd.batt_mv;
  bool low3s    = batt3s_mv < LOW_BATTERY_WARN_mV;
  float rpm     = readMotorRpm();
  float speed   = rpmToKmh(rpm);
  float ir      = readCurrent(R_IS_PIN);
  float il      = readCurrent(L_IS_PIN);
 
  Serial.print("1S ADC: ");
  Serial.print("Raw ADC: ");
  Serial.println(analogRead(BATT_PIN));
 
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
 
// =========================================================================
// Wi-Fi AP: must come up BEFORE espNowSetup() so they share a channel
// =========================================================================
 
void wifiSetup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.printf("AP SSID: %s\n", AP_SSID);
  Serial.print("AP IP: ");      Serial.println(WiFi.softAPIP());
  Serial.printf("Channel: %d\n", AP_CHANNEL);
}
 
// =========================================================================
// Arduino entry points
// =========================================================================
 
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
 
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
 
  motorSetup();
  servoSetup();
  wifiSetup();
  espNowSetup();
  httpSetup();
}
 
void loop() {
  selectActiveCommand();
  ws.cleanupClients();
 
  static unsigned long lastTelem = 0;
  if (millis() - lastTelem > 500) {
    sendTelemetry();
    lastTelem = millis();
  }
  delay(10);
}