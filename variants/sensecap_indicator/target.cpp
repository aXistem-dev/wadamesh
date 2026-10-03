// SPDX-License-Identifier: GPL-3.0-or-later
#include <Arduino.h>

#include "target.h"
#include "IndicatorIo.h"
#include "IndicatorRadioHal.h"

IndicatorBoard board;

// The radio's NSS/RESET/BUSY/DIO1 sit on the TCA9535; the P_LORA_* values are
// virtual pins that IndicatorRadioHal resolves to expander bits. SCK/MISO/MOSI
// are real GPIOs on FSPI.
static SPIClass radioSpi(FSPI);
static IndicatorRadioHal radio_hal(radioSpi);
RADIO_CLASS radio = new Module(&radio_hal, P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
ClockFloorRTC rtc_clock(fallback_clock);

EnvironmentSensorManager sensors;

IndicatorDisplay display;
MomentaryButton user_btn(PIN_USER_BTN, 1000, true);

float indicatorTcxoVoltage() {
  return IndicatorIo::radioHasTcxo() ? 2.4f : 0.0f;
}

bool radio_init() {
  fallback_clock.begin();
  rtc_clock.begin(Wire);
  Serial.printf("[indicator] radio tcxo=%.1f\n", indicatorTcxoVoltage());
  // std_init starts FSPI on GPIO41/47/48; main.cpp runs display.begin() first,
  // and the panel init has handed GPIO41/48 back by then.
  return radio.std_init(&radioSpi);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);
}
