/*
 * PulseSensor_Pico_2_W_SerialPlotter.ino
 * Version: 1.0.0
 *
 * Raw analog PulseSensor waveform for Raspberry Pi Pico 2 W.
 * Open Arduino Serial Plotter at 115200 baud.
 *
 * Hardware: Raspberry Pi Pico 2 W, PulseSensor
 *
 * Wiring:
 *   PulseSensor Signal (Purple) - GP28 / ADC2
 *   PulseSensor Power (Red)     - 3.3V
 *   PulseSensor Ground (Black)  - GND
 *
 * This software is for creative and educational use.
 * It is not intended for medical use.
 */

const int SENSOR_PIN = 28;
const int LED_PIN = LED_BUILTIN;
const int THRESHOLD = 550;

int lastSensorValue = 0;
unsigned long lastPulseTime = 0;

void setup();
void loop();

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("PulseSensor_Pico_2_W_SerialPlotter.ino");
  Serial.println("Signal:");
}

void loop() {
  int signal = analogRead(SENSOR_PIN);

  if (signal > THRESHOLD && signal > lastSensorValue && millis() - lastPulseTime > 300) {
    lastPulseTime = millis();
    digitalWrite(LED_PIN, HIGH);
    delay(20);
    digitalWrite(LED_PIN, LOW);
  }

  Serial.print("Signal:");
  Serial.println(signal);

  lastSensorValue = signal;
  delay(20);
}
