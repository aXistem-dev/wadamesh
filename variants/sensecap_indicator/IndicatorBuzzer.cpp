// SPDX-License-Identifier: GPL-3.0-or-later
#include "IndicatorBuzzer.h"

#include "Rp2040Uart.h"

// A map tile write holds the link for a whole file; a beep that cannot get it
// this soon is skipped rather than sounding late and holding up the next ones.
static constexpr uint32_t kLinkWaitMs = 500;

bool indicatorBeep(uint16_t ms) {
  rp2040::Msg m;
  m.kind = rp2040::Kind::Beep;
  m.beep_ms = ms;
  return Rp2040Uart().post(m, kLinkWaitMs);
}
