// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdint.h>

#include "IndicatorExpander.h"

namespace IndicatorIo {

bool begin();
bool ready();
indicator::Tca9535& chip();

// One recursive mutex over Wire on SDA39/SCL40. Every Wire user on that bus takes
// it; each expander transfer already does. Hold it across a sequence of chip()
// calls that must not interleave with another task's.
class BusLock {
public:
  BusLock();
  ~BusLock();
  BusLock(const BusLock&) = delete;
  BusLock& operator=(const BusLock&) = delete;
};

// The radio strap, read in begin(): true for the TCXO radio, false for the crystal.
bool radioHasTcxo();

}  // namespace IndicatorIo
