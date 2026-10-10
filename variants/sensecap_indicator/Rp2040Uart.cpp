// SPDX-License-Identifier: GPL-3.0-or-later
#include "Rp2040Uart.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <soc/soc.h>
#include <soc/usb_serial_jtag_reg.h>

#include <new>

namespace {

constexpr int kRxPin = 20;
constexpr int kTxPin = 19;
constexpr uint32_t kBaud = 2000000;
// Room for a few full frames while no call is reading.
constexpr size_t kRxBuffer = 16384;

SemaphoreHandle_t linkMutex() {
  static StaticSemaphore_t storage;
  static const SemaphoreHandle_t handle = xSemaphoreCreateRecursiveMutexStatic(&storage);
  return handle;
}

class SerialTransport : public rp2040::Transport {
public:
  size_t write(const uint8_t* data, size_t n) override { return Serial2.write(data, n); }

  // Polls once a tick: the caller's task sleeps instead of spinning while the
  // bridge works on a request.
  size_t read(uint8_t* buf, size_t n, uint32_t timeout_ms) override {
    uint32_t start = millis();
    for (;;) {
      int avail = Serial2.available();
      if (avail > 0) return Serial2.read(buf, n < (size_t)avail ? n : (size_t)avail);
      if (millis() - start >= timeout_ms) return 0;
      vTaskDelay(1);
    }
  }

  uint32_t nowMs() override { return millis(); }
};

SerialTransport s_transport;
// About 41 KB of buffers: allocated once, in PSRAM when there is some.
rp2040::LinkCore* s_core = nullptr;
bool s_ready = false;

}  // namespace

Rp2040Uart::Lock::Lock() { xSemaphoreTakeRecursive(linkMutex(), portMAX_DELAY); }
Rp2040Uart::Lock::~Lock() { xSemaphoreGiveRecursive(linkMutex()); }

bool Rp2040Uart::begin() {
  Lock lock;
  s_ready = false;
  if (!s_core) {
    void* mem = heap_caps_malloc(sizeof(rp2040::LinkCore), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mem) mem = heap_caps_malloc(sizeof(rp2040::LinkCore), MALLOC_CAP_8BIT);
    if (!mem) {
      Serial.println("[indicator] rp2040: no bridge (out of memory)");
      return false;
    }
    s_core = new (mem) rp2040::LinkCore(s_transport);
    // replies the bridge still owes a previous run must not match our ids
    s_core->setNextId(1 + esp_random() % rp2040::kMaxId);
  }

  // GPIO19/20 are the USB PHY's pads: release them to the UART
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
  Serial2.setRxBufferSize(kRxBuffer);
  Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);

  uint32_t version = 0;
  for (int attempt = 0; attempt < 2 && !s_ready; ++attempt) {
    s_ready = s_core->ping(version);
  }
  if (s_ready) {
    Serial.printf("[indicator] rp2040: bridge v%lu\n", (unsigned long)version);
  } else if (version != 0) {
    Serial.printf("[indicator] rp2040: no bridge (speaks v%lu, want v%lu)\n", (unsigned long)version,
                  (unsigned long)rp2040::kVersion);
  } else {
    Serial.println("[indicator] rp2040: no bridge (no reply)");
  }
  return s_ready;
}

bool Rp2040Uart::call(const rp2040::Msg& req, rp2040::Msg& resp, uint32_t timeout_ms) {
  Lock lock;
  if (!s_ready) {
    resp = rp2040::Msg{};
    return false;
  }
  return s_core->call(req, resp, timeout_ms);
}

int Rp2040Uart::callMany(const rp2040::Msg* reqs, rp2040::Msg* resps, int n, int window,
                         uint32_t timeout_ms) {
  Lock lock;
  if (!s_ready) return 0;
  return s_core->callMany(reqs, resps, n, window, timeout_ms);
}

uint32_t Rp2040Uart::helloCount() {
  Lock lock;
  return s_core ? s_core->helloCount() : 0;
}
