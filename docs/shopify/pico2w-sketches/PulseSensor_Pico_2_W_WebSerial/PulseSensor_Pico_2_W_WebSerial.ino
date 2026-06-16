/*
 * PulseSensor_Pico_2_W_WebSerial.ino
 * Version: 1.0.0
 *
 * Raspberry Pi Pico 2 W USB Serial / Web Serial example.
 * Streams PulseSensor data as newline-delimited CSV text for browser
 * Web Serial dashboards or any serial monitor.
 *
 * Hardware: Raspberry Pi Pico 2 W, PulseSensor
 *
 * Wiring:
 *   PulseSensor Signal (Purple) - GP28 / ADC2
 *   PulseSensor Power (Red)     - 3.3V
 *   PulseSensor Ground (Black)  - GND
 *
 * Serial settings:
 *   Baud: 115200
 *   Data line format: signal,bpm,ibi,beat
 *
 * GP28 / ADC2 is used here so the PulseSensor signal does not use
 * ADC0 or ADC1, which are commonly used by the 52Pi PicoDeck joystick.
 *
 * Copyright World Famous Electronics LLC - see LICENSE
 * Contributors:
 *   Joel Murphy, https://pulsesensor.com
 *   Yury Gitman, https://pulsesensor.com
 *
 * Licensed under the MIT License, a copy of which
 * should have been included with this software.
 *
 * This software is for creative and educational use.
 * It is not intended for medical use.
 */

#include "PulseSensor_Pico_2_W_WebSerial_Impl.h"
