// RoadrunnerIO/src/main.cpp — originally from Mirza, ported to Arduino-ESP32 core 3.x.
//
// PORTING NOTE (core 2.x -> 3.x): the LEDC API changed from channel-based to
// pin-based. ledcSetup()/ledcAttachPin() no longer exist, and ledcWrite() now
// takes a GPIO instead of a channel. The old code compiled on core 2.x only;
// on 3.x, ledcWrite(RPWM_CHANNEL, ...) would have written to "the channel
// attached to GPIO 4" -- nothing -- and the motors would have stayed silent
// with no error. The ESP-NOW receive callback signature changed too.
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
// TODO: every group in the course is running this sketch. Give yours a unique
// SSID or you will connect your phone to somebody else's car.
const char* AP_SSID     = "RC-Car-XX";          // Change name to something unique
const char* AP_PASSWORD = "maskinelement";
const int   AP_CHANNEL  = 1;                    // ESP-NOW will inherit this channel

// Motor + servo pins (see the Electronics and Wiring page)
const int RPWM_PIN      = 16;
const int LPWM_PIN      = 17;
const int SERVO_PIN     = 13;
const int MOTOR_EN_PIN  = 19;                   // R_EN + L_EN tied together to this GPIO (NOT 12, strapping pin)
const int BATT_PIN      = 34;                   // 3S divider midpoint, ADC1, input-only

// HARDWARE NOTE: fit a ~10k pulldown from MOTOR_EN_PIN to GND. Between reset
// and motorSetup() every GPIO floats, and a floating R_EN/L_EN can let the
// BTS7960 drive the motor for a few hundred ms at power-up. The pulldown is
// the only thing that makes "motors off at boot" true in hardware.

// Optional BTS7960 current sense: set to -1 to disable until you wire R_IS / L_IS.
// When wired, each pin outputs ~ I_motor / 8500 amps to GND through a sense resistor.
const int R_IS_PIN      = -1;                   // e.g. 32 once wired (ADC1)
const int L_IS_PIN      = -1;                   // e.g. 33 once wired (ADC1)

// PWM. On core 3.x the LEDC channel is allocated for us by ledcAttach(), so
// there are no channel numbers to keep clear of other libraries any more.
const int MOTOR_FREQ    = 20000;                // 20 kHz, above audible
const int MOTOR_RES     = 8;                    // 0..255 duty
const int SERVO_FREQ    = 50;
const int SERVO_RES     = 16;                   // 65 536 ticks per 20 ms

// Servo calibration: replace with YOUR measured end stops.
// SERVO_CENTER_US is the trim: set it to the pulse width where the wheels
// actually point straight ahead. It is NOT necessarily the midpoint of
// MIN/MAX -- servo horns are never mounted perfectly.
const int SERVO_MIN_US    = 500;
const int SERVO_CENTER_US = 1490;
const int SERVO_MAX_US    = 2480;

// 3S battery calibration: replace with YOUR two-point readings
const long ADC_AT_9V   = 1880;                  // raw ADC at 9.0 V
const long ADC_AT_12V6 = 2630;                  // raw ADC at 12.6 V
const long LOW_BATTERY_WARN_mV = 9600;          // 3S cutoff (~3.2 V/cell)

// Drivetrain: used to convert motor RPM into vehicle km/h
const float WHEEL_DIAM_M  = 0.080f;             // wheel diameter in meters, measure yours
const float GEAR_RATIO    = 1.0f;               // motor turns per wheel turn (1.0 = direct drive)

// Command-source arbitration windows
const unsigned long ESPNOW_PRIO_MS = 200;       // joystick wins if last packet < 200 ms ago
const unsigned long FAILSAFE_MS    = 500;       // both stale > 500 ms -> motor off

// Control shaping
const int CONTROL_PERIOD_MS = 20;               // one loop() iteration = 50 Hz
const int THROTTLE_DEADZONE = 12;               // ignore stick noise around centre
const int STEERING_DEADZONE = 8;
// Max throttle change per 20 ms tick. Slamming the stick from full reverse to
// full forward shorts the back-EMF through the BTS7960: a big current spike,
// a brown-out reset, and a lot of heat. 12 counts/tick ramps 0 -> full in
// ~0.4 s and a full reversal in ~0.9 s, which the battery can actually supply.
const int THROTTLE_SLEW_PER_TICK = 12;

// =========================================================================
// State: written by callbacks, read by loop()
// =========================================================================

typedef struct {
  int16_t steering;   // -255 .. +255
  int16_t throttle;   // -255 .. +255
  bool    button;
  int16_t batt_mv;    // XIAO 1S battery, mV, filled in by the optional remote
} ControlData;

// espnowCmd is written by the Wi-Fi task, webCmd by the AsyncTCP task, and
// both are read by loop() on another core. A struct copy is not atomic, so
// without a lock you can read the steering from one packet and the throttle
// from the next. cmdMux makes each snapshot consistent.
static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

ControlData espnowCmd = {0, 0, false, 0};       // last ESP-NOW packet
ControlData webCmd    = {0, 0, false, 0};       // last WebSocket message
ControlData activeCmd = {0, 0, false, 0};       // whichever is currently driving

unsigned long lastEspNowMs = 0;
unsigned long lastWebMs    = 0;
// "Never received anything yet" is not the same as "received at millis()==0".
// Without these flags, millis() - lastWebMs is tiny right after boot, so the
// car reports source "joystick" then "web" for the first 500 ms instead of
// "failsafe".
bool espnowSeen = false;
bool webSeen    = false;

const char*   activeSource = "failsafe";

volatile bool motorsEnabled = false;            // off at boot, user has to enable from the UI
volatile int16_t appliedThrottle = 0;           // slew-limited value actually on the pins

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =========================================================================
// Battery monitor: two-point linear interpolation
// =========================================================================

long readBatteryMillivolts() {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(BATT_PIN);
  long raw = sum / 16;
  // Clamp: outside the calibrated span map() extrapolates, so a disconnected
  // divider (raw ~0) would report a large negative voltage rather than "flat".
  long mv = map(raw, ADC_AT_9V, ADC_AT_12V6, 9000, 12600);
  return constrain(mv, 0L, 13000L);
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
  appliedThrottle = 0;                          // do not resume mid-ramp later
}

void motorSetup() {
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);              // motors hardware-disabled at boot

  if (!ledcAttach(RPWM_PIN, MOTOR_FREQ, MOTOR_RES) ||
      !ledcAttach(LPWM_PIN, MOTOR_FREQ, MOTOR_RES)) {
    Serial.println("ledcAttach failed for the motor pins");
  }
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
}

void setMotorEnable(bool on) {
  motorsEnabled = on;
  // Called from the AsyncTCP task. The EN pin is written here rather than
  // deferred to loop() so that "disable" takes effect immediately instead of
  // up to one control tick later -- this is the hardware kill path.
  digitalWrite(MOTOR_EN_PIN, on ? HIGH : LOW);  // BTS7960 R_EN + L_EN
  if (!on) stopMotor();                         // belt-and-braces: also kill PWM
}

void servoSetup() {
  if (!ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES)) {
    Serial.println("ledcAttach failed for the servo pin");
  }
  ledcWrite(SERVO_PIN, usToServoDuty(SERVO_CENTER_US));
}

int applyDeadzone(int v, int dz) {
  return (v > -dz && v < dz) ? 0 : v;
}

void applyControl(const ControlData& cmd) {
  // Steering is applied even when the motors are disabled, so you can centre
  // and calibrate the servo on the bench without the car trying to drive off.
  // Mapped around SERVO_CENTER_US so the trim above actually does something.
  int s = applyDeadzone(constrain((int)cmd.steering, -255, 255), STEERING_DEADZONE);
  int us = (s >= 0) ? map(s, 0, 255, SERVO_CENTER_US, SERVO_MAX_US)
                    : map(s, -255, 0, SERVO_MIN_US, SERVO_CENTER_US);
  ledcWrite(SERVO_PIN, usToServoDuty(constrain(us, SERVO_MIN_US, SERVO_MAX_US)));

  if (!motorsEnabled) {                         // disarmed: never drive the bridge
    stopMotor();
    return;
  }

  // Throttle: ramp towards the target instead of jumping to it. Read the
  // shared value once into a local so the RPWM/LPWM pair below is always
  // computed from the same number, even if setMotorEnable() zeroes it from
  // the AsyncTCP task halfway through.
  int target = applyDeadzone(constrain((int)cmd.throttle, -255, 255), THROTTLE_DEADZONE);
  int next   = appliedThrottle;
  next += constrain(target - next, -THROTTLE_SLEW_PER_TICK, THROTTLE_SLEW_PER_TICK);
  appliedThrottle = (int16_t)next;

  // Split sign across RPWM and LPWM. Exactly one side is ever driven; driving
  // both at once is a shoot-through on the BTS7960.
  if (next > 0) {
    ledcWrite(LPWM_PIN, 0);
    ledcWrite(RPWM_PIN, next);
  } else {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, -next);
  }
}

// =========================================================================
// Command-source arbitration: joystick > web > failsafe
// =========================================================================

void selectActiveCommand() {
  unsigned long now = millis();

  // Take one consistent snapshot of both command sources.
  ControlData espSnap, webSnap;
  unsigned long espMs, webMs;
  bool espOk, webOk;
  portENTER_CRITICAL(&cmdMux);
  espSnap = espnowCmd;  espMs = lastEspNowMs;  espOk = espnowSeen;
  webSnap = webCmd;     webMs = lastWebMs;     webOk = webSeen;
  portEXIT_CRITICAL(&cmdMux);

  if (espOk && (now - espMs) < ESPNOW_PRIO_MS) {
    activeCmd    = espSnap;
    activeSource = "joystick";
  } else if (webOk && (now - webMs) < FAILSAFE_MS) {
    activeCmd    = webSnap;
    activeSource = "web";
  } else {
    activeCmd    = {0, 0, false, 0};
    activeSource = "failsafe";
    stopMotor();                                // cut immediately, do not ramp down
    // Centre the wheels too. Holding the last steering angle after losing the
    // link means a car that coasts away in a circle at full lock; straight is
    // the predictable failure.
    ledcWrite(SERVO_PIN, usToServoDuty(SERVO_CENTER_US));
    return;                                     // skip applyControl when failed-safe
  }
  applyControl(activeCmd);
}

// =========================================================================
// ESP-NOW receive callback: fires whenever a packet from the XIAO arrives
// =========================================================================

// Core 3.x passes an esp_now_recv_info_t* instead of a bare MAC pointer.
void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;                                   // add a MAC allow-list here if you want one
  if (len != (int)sizeof(ControlData)) return;  // ignore unexpected lengths
  portENTER_CRITICAL(&cmdMux);
  memcpy(&espnowCmd, data, sizeof(espnowCmd));
  lastEspNowMs = millis();
  espnowSeen   = true;
  portEXIT_CRITICAL(&cmdMux);
}

void espNowSetup() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowReceive);
}

// =========================================================================
// WebSocket: short text frames from the phone:
//   "steering,throttle"     = joystick state (-255 .. +255 each)
//   "en1" / "en0"           = motors-enabled toggle
// =========================================================================

void parseWebMessage(const uint8_t* data, size_t len) {
  // The old code did data[len] = 0, which writes one byte past the end of the
  // frame buffer AsyncWebSocket handed us -- a heap overflow that corrupts
  // whatever allocation follows it. Copy into our own buffer instead.
  char buf[32];
  if (len == 0 || len >= sizeof(buf)) return;
  memcpy(buf, data, len);
  buf[len] = '\0';

  if (buf[0] == 'e' && buf[1] == 'n') {
    setMotorEnable(buf[2] == '1');
    return;
  }

  int s, t;
  if (sscanf(buf, "%d,%d", &s, &t) != 2) return;

  portENTER_CRITICAL(&cmdMux);
  webCmd.steering = (int16_t)constrain(s, -255, 255);
  webCmd.throttle = (int16_t)constrain(t, -255, 255);
  webCmd.button   = false;
  lastWebMs       = millis();
  webSeen         = true;
  portEXIT_CRITICAL(&cmdMux);
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  (void)server; (void)client;
  switch (type) {
    case WS_EVT_DATA: {
      // WS_EVT_DATA fires per frame, not per message. Only act on a complete,
      // unfragmented text frame; otherwise a binary frame or the second half
      // of a split message gets parsed as if it were a command.
      const AwsFrameInfo* info = (const AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->message_opcode == WS_TEXT) {
        parseWebMessage(data, len);
      }
      break;
    }
    case WS_EVT_DISCONNECT:
      // Do not coast on the last command from a phone that has gone away.
      portENTER_CRITICAL(&cmdMux);
      webCmd  = {0, 0, false, 0};
      webSeen = false;
      portEXIT_CRITICAL(&cmdMux);
      break;
    default:
      break;
  }
}

// =========================================================================
// HTTP: serve the embedded controller page from PROGMEM
// =========================================================================

void httpSetup() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    // Streams the page straight out of flash. send_P() is deprecated in
    // ESPAsyncWebServer 3.x AND, on the ESP32, now forwards to the overload
    // that copies the whole 27 KB page into a heap String on every request.
    // This overload builds an AsyncProgmemResponse (pointer + offset) instead,
    // which costs no heap at all.
    req->send(200, "text/html", (const uint8_t*)CONTROLLER_HTML,
              sizeof(CONTROLLER_HTML) - 1);
  });
  server.begin();
}

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
  return wheelRpm * circ_m * 60.0f / 1000.0f;   // m/min -> km/h
}

// BTS7960 current sense: returns 0 unless you wire R_IS / L_IS to ADC pins.
// The chip pulls roughly I_motor / 8500 amps out of each IS pin; with a 1 kOhm
// sense resistor to GND that is ~118 mV per ampere of motor current.
// (The original comment said 0.118 mV/A -- off by a factor of 1000. The code
// was right, the comment was not.)
// analogReadMilliVolts() applies the chip's factory ADC calibration, which is
// a lot more accurate than raw * 3.3/4095 -- the ESP32 ADC is not linear and
// does not actually reach 3.3 V at full scale.
float readCurrent(int pin) {
  if (pin < 0) return 0.0f;
  float v = analogReadMilliVolts(pin) / 1000.0f;
  // I_motor = V_sense / R_sense * 8500.   With R_sense = 1 kOhm:
  return v / 1000.0f * 8500.0f;
}

// =========================================================================
// Telemetry: pushed to all connected web clients every 500 ms
// =========================================================================

void sendTelemetry() {
  if (ws.count() == 0) return;                  // nobody listening, skip the work

  long batt3s_mv = readBatteryMillivolts();
  portENTER_CRITICAL(&cmdMux);
  long batt1s_mv = espnowCmd.batt_mv;           // sent by the XIAO transmitter
  portEXIT_CRITICAL(&cmdMux);

  bool  low3s = batt3s_mv < LOW_BATTERY_WARN_mV;
  float rpm   = readMotorRpm();
  float speed = rpmToKmh(rpm);
  float ir    = readCurrent(R_IS_PIN);
  float il    = readCurrent(L_IS_PIN);

  // Built with snprintf rather than String concatenation: the old version
  // allocated and freed ~10 heap blocks twice a second, which fragments the
  // heap over a long run and eventually starves AsyncTCP.
  char json[256];
  snprintf(json, sizeof(json),
           "{\"batt3s_v\":%.2f,\"batt1s_v\":%.2f,\"batt_low\":%s,"
           "\"motor_rpm\":%.0f,\"speed_kmh\":%.1f,"
           "\"current_r\":%.2f,\"current_l\":%.2f,"
           "\"enabled\":%s,\"source\":\"%s\"}",
           batt3s_mv / 1000.0f, batt1s_mv / 1000.0f, low3s ? "true" : "false",
           rpm, speed, ir, il,
           motorsEnabled ? "true" : "false", activeSource);
  ws.textAll(json);

  if (low3s) Serial.println("!! LOW 3S BATTERY (< 9.6 V) !!");
}

// =========================================================================
// Wi-Fi AP: must come up BEFORE espNowSetup() so they share a channel
// =========================================================================

void wifiSetup() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL)) {
    Serial.println("softAP failed to start");
  }
  // Modem sleep adds tens of ms of jitter to every control packet. This is a
  // real-time control link on USB/battery power, so keep the radio awake.
  WiFi.setSleep(false);
  Serial.printf("AP SSID: %s\n", AP_SSID);
  Serial.print("AP IP: ");      Serial.println(WiFi.softAPIP());
  Serial.printf("Channel: %d\n", AP_CHANNEL);
}

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

  // Enabling is only possible from the web UI, so if every phone has gone
  // away there is nobody left who can press stop. Disarm.
  if (motorsEnabled && ws.count() == 0) setMotorEnable(false);

  static unsigned long lastTelem   = 0;
  static unsigned long lastCleanup = 0;
  unsigned long now = millis();

  if (now - lastTelem > 500) {
    sendTelemetry();
    lastTelem = now;
  }
  // Once a second is plenty; at 50 Hz this was pure overhead.
  if (now - lastCleanup > 1000) {
    ws.cleanupClients();
    lastCleanup = now;
  }
  delay(CONTROL_PERIOD_MS);
}
