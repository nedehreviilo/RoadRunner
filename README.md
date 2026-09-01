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

