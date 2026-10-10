// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// The SenseCAP Indicator's buzzer is wired to the RP2040, and the bridge
// firmware drives it at one fixed pitch for a given time. The bridge sends no
// reply to a beep, so this does not wait; without a bridge nothing sounds.

#include <stdint.h>

// Sounds the buzzer for ms milliseconds; 0 stops a running beep. False when
// the bridge is not up, the link stayed busy for half a second, or the frame
// did not go out.
bool indicatorBeep(uint16_t ms);
