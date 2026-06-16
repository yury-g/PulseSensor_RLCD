/*
 * PulseSensor_Pico_2_W_BPM.ino
 * Version: 1.0.0
 *
 * Basic Raspberry Pi Pico 2 W workbench example.
 * Displays beats per minute in the Serial Monitor.
 *
 * Hardware: Raspberry Pi Pico 2 W, PulseSensor
 *
 * Wiring:
 *   PulseSensor Signal (Purple) - GP28 / ADC2
 *   PulseSensor Power (Red)     - 3.3V
 *   PulseSensor Ground (Black)  - GND
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

#include "PulseSensor_Pico_2_W_BPM_Impl.h"
