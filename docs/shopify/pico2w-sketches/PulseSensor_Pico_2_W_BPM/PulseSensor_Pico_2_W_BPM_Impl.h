#ifndef PULSESENSOR_PICO_2_W_BPM_IMPL_H
#define PULSESENSOR_PICO_2_W_BPM_IMPL_H

#include <PulseSensorPlayground.h>

const int PULSE_INPUT = 28;
const int PULSE_BLINK = LED_BUILTIN;
const int THRESHOLD = 550;

PulseSensorPlayground pulseSensor;

void setup() {
  Serial.begin(115200);
  Serial.println("PulseSensor_Pico_2_W_BPM.ino");
  Serial.println("PulseSensor.com");
  Serial.println("PulseSensor signal: GP28 / ADC2");

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.setThreshold(THRESHOLD);

  bool pulseSensorStarted = pulseSensor.begin();
  if (!pulseSensorStarted) {
    Serial.println("PulseSensor Playground did not start.");
    Serial.println("Check board support and installed timer libraries.");
  }
}

void loop() {
  int bpm = pulseSensor.getBeatsPerMinute();

  if (pulseSensor.sawStartOfBeat()) {
    Serial.print("BPM: ");
    Serial.println(bpm);
  }

  delay(20);
}

#endif
