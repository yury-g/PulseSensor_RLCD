#ifndef PULSESENSOR_PICO_2_W_WEBSERIAL_IMPL_H
#define PULSESENSOR_PICO_2_W_WEBSERIAL_IMPL_H

#include <PulseSensorPlayground.h>

const int PULSE_INPUT = 28;
const int PULSE_BLINK = LED_BUILTIN;
const int THRESHOLD = 550;
const unsigned long SERIAL_BAUD = 115200;
const unsigned long OUTPUT_INTERVAL_MS = 20;
const unsigned long SERIAL_WAIT_MS = 3000;
const bool PRINT_STARTUP_COMMENTS = true;

PulseSensorPlayground pulseSensor;

bool pulseSensorStarted = false;
bool beatSinceLastOutput = false;
unsigned long lastOutputMs = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);

  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < SERIAL_WAIT_MS)) {
    delay(10);
  }

  if (PRINT_STARTUP_COMMENTS) {
    Serial.println("# PulseSensor_Pico_2_W_WebSerial.ino");
    Serial.println("# PulseSensor.com");
    Serial.println("# Board: Raspberry Pi Pico 2 W");
    Serial.println("# PulseSensor signal: GP28 / ADC2");
    Serial.println("# Format: signal bpm ibi beat");
  }

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.setThreshold(THRESHOLD);

  pulseSensorStarted = pulseSensor.begin();
  if (!pulseSensorStarted) {
    if (PRINT_STARTUP_COMMENTS) {
      Serial.println("# ERROR: PulseSensor Playground did not start.");
      Serial.println("# Check board support and installed timer libraries.");
    }
    return;
  }

  if (PRINT_STARTUP_COMMENTS) {
    Serial.println("# Streaming CSV at 115200 baud.");
  }
}

void loop() {
  if (!pulseSensorStarted) {
    delay(1000);
    return;
  }

  if (pulseSensor.sawStartOfBeat()) {
    beatSinceLastOutput = true;
  }

  unsigned long now = millis();
  if (now - lastOutputMs >= OUTPUT_INTERVAL_MS) {
    lastOutputMs = now;

    Serial.print(pulseSensor.getLatestSample());
    Serial.print(',');
    Serial.print(pulseSensor.getBeatsPerMinute());
    Serial.print(',');
    Serial.print(pulseSensor.getInterBeatIntervalMs());
    Serial.print(',');
    Serial.println(beatSinceLastOutput ? 1 : 0);

    beatSinceLastOutput = false;
  }
}

#endif
