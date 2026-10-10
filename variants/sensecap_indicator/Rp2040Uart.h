// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// The RP2040 bridge link on the Indicator: LinkCore over UART2 (Serial2) at
// 2 Mbaud, ESP TX GPIO19 / RX GPIO20, the pins the S3's USB PHY would
// otherwise claim. One link per board, so every instance shares it. Calls from
// different tasks take turns on a recursive mutex.

#include <stdint.h>

#include "Rp2040Link.h"

class Rp2040Uart {
public:
  // Opens the UART and pings the bridge, twice at most. True when it answers
  // in our protocol version; logs the outcome once either way.
  bool begin();

  // LinkCore::call and callMany under the link's mutex; false / 0 before a
  // successful begin(). A response's file data lives in the link until its
  // next call: a caller that reads it after the call returns holds a Lock
  // across both, or another task's call may overwrite it.
  bool call(const rp2040::Msg& req, rp2040::Msg& resp,
            uint32_t timeout_ms = rp2040::kReplyTimeoutMs);
  int callMany(const rp2040::Msg* reqs, rp2040::Msg* resps, int n, int window = 3,
               uint32_t timeout_ms = rp2040::kReplyTimeoutMs);
  // LinkCore::post under the link's mutex, waiting at most wait_ms for it;
  // false when the link stays busy that long, or before a successful begin().
  bool post(const rp2040::Msg& req, uint32_t wait_ms);
  uint32_t helloCount();

  // Holds the link's mutex; it is recursive, so calls inside are fine.
  class Lock {
  public:
    Lock();
    ~Lock();
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
  };
};
