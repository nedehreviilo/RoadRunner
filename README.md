# RoadRunner
A repo for the RC-Build microcontroller and controls to the car for the course and project in machine elements at Jönköping University.
Authors: Hjalmar Carlsson, Johan Myrberg, Oliver Hedén

To work the project you need platformio installed in your IDE. The project is built for the ESP32 microcontroller and uses the Arduino framework.
RoadRunner/
├── platformio.ini
└── src/
    ├── main.cpp        <- the sketch below
    └── controller.h    <- the web page, as a PROGMEM string

lib_deps =
    ESP32Async/ESPAsyncWebServer@^3.6.0
    ESP32Async/AsyncTCP@^3.3.0

The HTML, CSS and JavaScript live inside controller.h as one const char[] PROGMEM string, so there is no filesystem image to upload. One pio run --target upload covers both files.

    ┌─────────────────────────────────────┐
    │         ESP32 DevKit (CAR)          │
    │  ┌────────────────┐  ┌───────────┐  │   ┌────────────┐
    │  │ Wi-Fi AP +     │  │ command   │←─WebSocket │ Phone │
    │  │ HTTP + WS      │  │ state     │  │   │ (web UI)   │
    │  └────────────────┘  └─────┬─────┘  │   └────────────┘
    │                            │        │         ▲
    │                            ▼        │         │
    │                      ┌───────────┐  │         │
    │                      │ Servo +   │  │         │
    │                      │ BTS7960   │  │         │
    │                      └───────────┘  │         │
    │  ┌────────────────────────┐         │         │
    │  │ Battery ADC + RPM/speed│─── telemetry ─────┘
    │  └────────────────────────┘         │
    └─────────────────────────────────────┘

One rule governs the whole file. Callbacks never touch the motor. The WebSocket handler copies the incoming command into a variable, stamps a timestamp and returns. loop() is the only place that writes PWM. This keeps the failsafe in one readable place, avoids reentrancy between an asynchronous callback and the main thread, and keeps the callbacks short enough that the watchdog stays happy.

Everything you tune per car sits at the top of main.cpp.

The sketch below ships with the simple version of applyControl(). Positive throttle drives RPWM, negative throttle drives LPWM, and zero writes both to zero. 
        V+ ──┬──────────┬── V+
            │          │
            S1         S3
            │          │
            ├── MOTOR ─┤
            │          │
            S2         S4
            │          │
        GND ─┴──────────┴── GND

Stick|RPWM|LPWM|Resulting mode
---|---|---|---|
Forward|duty|0|Forward
Centred|0|0|Brake, because EN is still high
Back|0|duty|Reverse, immediately, at any speed
That is a working baseline, not the finished behaviour. The project requires:

Stick|Car moving forward|Car stationary
---|---|---|
Forward|Drive|Drive
Centred|Coast|Coast
Back|Brake|Reverse

Making the car coast on a centred stick means dropping EN low, not just zeroing the PWM. Making the car brake instead of reversing means knowing whether it is still moving, which means you need a speed signal or a modelled estimate. Implementing that is your work, and it is the part where the H-bridge theory earns its place.

Constants you must change
Everything you tune per car sits at the top of main.cpp.

Constant	What it affects
AP_SSID	The network name your phone connects to. Make it unique to your group
AP_PASSWORD	The network password
SERVO_MIN_US, SERVO_MAX_US	Steering end stops. Set these before the linkage can jam
ADC_AT_9V, ADC_AT_12V6	Battery divider calibration from the wiring page
LOW_BATTERY_WARN_mV	Cutoff threshold, 9900 for a 3S pack
WHEEL_DIAM_M	Wheel diameter in metres
GEAR_RATIO	Motor turns per wheel turn
❗ The gear ratio is a shared number

GEAR_RATIO and WHEEL_DIAM_M are the same numbers you dimension the transmission around. Leave them at the defaults and the speed telemetry is fiction, which makes it useless for checking whether you actually hit 10 km/h.

Throttle logic
The sketch below ships with the simple version of applyControl(). Positive throttle drives RPWM, negative throttle drives LPWM, and zero writes both to zero. Read that against the mode table on the H-Bridge page and you can see what it actually does:

Stick	RPWM	LPWM	Resulting mode
Forward	duty	0	Forward
Centred	0	0	Brake, because EN is still high
Back	0	duty	Reverse, immediately, at any speed
That is a working baseline, not the finished behaviour. The project requires:

Stick	Car moving forward	Car stationary
Forward	Drive	Drive
Centred	Coast	Coast
Back	Brake	Reverse
Making the car coast on a centred stick means dropping EN low, not just zeroing the PWM. Making the car brake instead of reversing means knowing whether it is still moving, which means you need a speed signal or a modelled estimate. Implementing that is your work, and it is the part where the H-bridge theory earns its place.

What each part does
Function	Purpose
readBatteryMillivolts()	Averages 16 ADC samples, then maps the raw value through your two calibration points
motorSetup()	Configures the LEDC channels and drives MOTOR_EN_PIN low, so the bridge boots disabled
setMotorEnable(bool)	The only place the enable pin moves. Also zeroes the PWM on the way down
applyControl()	Turns a command into PWM writes and a servo pulse width
stopMotor()	The failsafe primitive
rpmToKmh()	Applies GEAR_RATIO and WHEEL_DIAM_M
readMotorRpm()	Returns 0 until you fit an encoder
readCurrent(pin)	Reads a BTS7960 IS pin and divides by 8500. Returns 0 if the pin is set to -1
selectActiveCommand()	Picks which input is driving, and cuts the motor if none is fresh
sendTelemetry()	Builds one JSON object per cycle and broadcasts it to every connected phone
wifiSetup()	Starts the access point. Must run before espNowSetup()
Bring-up
Set the constants above and upload.
Open the serial monitor. You should see the SSID, the IP address and the channel.
Connect your phone to RC-Car-XX with the password maskinelement.
Open http://192.168.4.1.
Check the telemetry updates every half second and the battery voltage matches a multimeter reading.
Put the car on a stand so the wheels are clear of the bench.
Tap the button to enable the motors. It turns green.
Test throttle and steering on the stand before the car ever touches the floor.

The motors are disabled at boot and stay disabled until you tap the button. The first time you enable them, have the car on a stand. A 775 motor on a 3S pack will pull the car off a bench before you can reach the switch.

It is the web page the car serves to your phone: markup, styles, the nipplejs joystick library and the application script, wrapped in one PROGMEM string. The library is vendored in full, so there is no download and no PlatformIO dependency to add.

The R"rawliteral( ... )rawliteral" wrapper is a C++11 raw string literal. Everything between the parentheses is taken verbatim, with no escape processing, which is what you want when the content is full of quotes and backslashes.

The joystick library sits on one very long line about a third of the way down. Partial copies fail silently: the page loads, the telemetry updates, and the sticks never appear. If that happens, you truncated this file.

controller.h
The web page the car serves to your phone lives in src/controller.h as one PROGMEM string. It is long, mostly vendored library code, and nothing in it needs changing, so it has a page of its own: controller.h. Copy it whole into your project.

Two kinds of frame travel up the WebSocket: "steering,throttle" at 50 Hz, and "en1" or "en0" when someone taps the enable button.

The listing also contains ESP-NOW code and a command arbiter. Those belong to the optional physical remote. Leave them in place if you are driving from a phone. They cost nothing and the code is proven as it stands.