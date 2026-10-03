// SPDX-License-Identifier: GPL-3.0-or-later
#include "IndicatorRadioHal.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "IndicatorIo.h"

namespace {

using indicator::expanderBit;
using indicator::isExpanderPin;
namespace pins = indicator::pins;

constexpr uint8_t kExpanderIntGpio = 42;       // TCA9535 /INT, open drain, active low
constexpr uint32_t kDispatchStackBytes = 3072;
constexpr TickType_t kLostIntPoll = pdMS_TO_TICKS(50);
constexpr uint32_t kResetSettleMs = 20;        // without it begin() reports CHIP_NOT_FOUND

// Set in OUTPUT and OUTPUT_OPEN_DRAIN, clear in INPUT and its pull variants.
constexpr uint32_t kOutputModeBit = OUTPUT & ~INPUT;

TaskHandle_t s_dispatch = nullptr;

// Only wakes the task: the expander read needs I2C and the bus mutex.
void IRAM_ATTR onExpanderInt() {
  BaseType_t woken = pdFALSE;
  if (s_dispatch) vTaskNotifyGiveFromISR(s_dispatch, &woken);
  if (woken) portYIELD_FROM_ISR();
}

// Never touches RadioLib or SPI. The input read clears /INT and feeds the
// observer, which calls the radio callback on a DIO1 rise. The timeout re-reads
// anyway, in case /INT fell while the ISR was not yet attached or a change
// landed inside another reader's transfer.
void dispatchTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, kLostIntPoll);
    IndicatorIo::BusLock lock;
    // Both input bytes: /INT clears only when the port that changed is read, so a
    // port-0-only read would leave a port-1 change holding GPIO42 low for good.
    // A port-1 pin read covers both bytes and still reports port 0 to the observer.
    bool strap = false;
    IndicatorIo::chip().readPin(pins::kRadioStrap, strap);
  }
}

}  // namespace

IndicatorRadioHal::IndicatorRadioHal(SPIClass& spi, uint32_t spiFreq)
    : ArduinoHal(spi, SPISettings(spiFreq, MSBFIRST, SPI_MODE0)) {}

// Module::init() runs this from SX126x::begin(), before any pin call. The
// observer goes in here rather than in the constructor: the expander object
// lives in another translation unit and may not be constructed yet at static
// init time.
void IndicatorRadioHal::init() {
  ArduinoHal::init();
  IndicatorIo::BusLock lock;
  IndicatorIo::chip().setPort0Observer(&IndicatorRadioHal::onPort0, this);
}

// Runs inside a port-0 read, so under the BusLock the reader holds.
void IndicatorRadioHal::onPort0(uint8_t value, void* ctx) {
  static_cast<IndicatorRadioHal*>(ctx)->_dio1.onPort0(value);
}

// An output keeps its current latch: the NSS high set at boot must survive
// RadioLib's pinMode(cs, OUTPUT).
void IndicatorRadioHal::pinMode(uint32_t pin, uint32_t mode) {
  if (!isExpanderPin(pin)) {
    ArduinoHal::pinMode(pin, mode);
    return;
  }
  const uint8_t bit = expanderBit(pin);
  auto& io = IndicatorIo::chip();
  IndicatorIo::BusLock lock;
  if (mode & kOutputModeBit) {
    io.setOutput(bit, (io.output(bit >> 3) & (1U << (bit & 7))) != 0);
  } else {
    io.setInput(bit);
  }
}

void IndicatorRadioHal::digitalWrite(uint32_t pin, uint32_t value) {
  if (!isExpanderPin(pin)) {
    ArduinoHal::digitalWrite(pin, value);
    return;
  }
  const uint8_t bit = expanderBit(pin);
  {
    IndicatorIo::BusLock lock;
    IndicatorIo::chip().setOutput(bit, value != 0);
  }
  if (bit == pins::kLoraReset && value) ::delay(kResetSettleMs);
}

// A BUSY read is a port-0 read, so it samples DIO1 too: a rise that lands while
// RadioLib waits on BUSY fires the callback from here, and the read that clears
// /INT is never a lost edge.
uint32_t IndicatorRadioHal::digitalRead(uint32_t pin) {
  if (!isExpanderPin(pin)) return ArduinoHal::digitalRead(pin);
  bool high = false;
  IndicatorIo::BusLock lock;
  if (!IndicatorIo::chip().readPin(expanderBit(pin), high)) return LOW;
  return high ? HIGH : LOW;
}

// Only DIO1 has an edge path, and Dio1Edge only knows rising edges, which is
// what SX126x::setDio1Action asks for.
void IndicatorRadioHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void),
                                        uint32_t mode) {
  if (!isExpanderPin(interruptNum)) {
    ArduinoHal::attachInterrupt(interruptNum, interruptCb, mode);
    return;
  }
  if (expanderBit(interruptNum) != pins::kLoraDio1) return;
  {
    IndicatorIo::BusLock lock;
    _dio1.arm(interruptCb);
  }
  startDispatch();
}

// The dispatch task keeps reading while disarmed, so Dio1Edge keeps tracking the
// level and a later re-arm with DIO1 already high is no edge.
void IndicatorRadioHal::detachInterrupt(uint32_t interruptNum) {
  if (!isExpanderPin(interruptNum)) {
    ArduinoHal::detachInterrupt(interruptNum);
    return;
  }
  if (expanderBit(interruptNum) != pins::kLoraDio1) return;
  IndicatorIo::BusLock lock;
  _dio1.disarm();
}

uint32_t IndicatorRadioHal::pinToInterrupt(uint32_t pin) {
  return isExpanderPin(pin) ? pin : ArduinoHal::pinToInterrupt(pin);
}

void IndicatorRadioHal::startDispatch() {
  if (s_dispatch) return;
  if (xTaskCreatePinnedToCore(dispatchTask, "ind_dio1", kDispatchStackBytes, nullptr,
                              configMAX_PRIORITIES - 2, &s_dispatch,
                              ARDUINO_RUNNING_CORE) != pdPASS) {
    s_dispatch = nullptr;
    Serial.println("[indicator] radio hal: dispatch task not created");
    return;
  }
  ::pinMode(kExpanderIntGpio, INPUT_PULLUP);
  ::attachInterrupt(kExpanderIntGpio, onExpanderInt, FALLING);
  Serial.printf("[indicator] radio hal: dio1 via expander /INT on GPIO%u\n",
                (unsigned)kExpanderIntGpio);
}
