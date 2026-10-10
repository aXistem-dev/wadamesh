// SPDX-License-Identifier: GPL-3.0-or-later
#include "IndicatorBoard.h"

#include <Arduino.h>

#include "IndicatorIo.h"
#include "IndicatorSdFs.h"

void IndicatorBoard::begin() {
  ESP32Board::begin();   // starts Wire on PIN_BOARD_SDA/PIN_BOARD_SCL
  if (!IndicatorIo::begin()) Serial.println("[indicator] expander unavailable");
  // The RP2040 is out of reset now (expander P1.0 high). Map tiles come from
  // the microSD in its slot when its bridge answers; up to about 1 s at boot
  // when it does not.
  indicatorSdBegin();
}
