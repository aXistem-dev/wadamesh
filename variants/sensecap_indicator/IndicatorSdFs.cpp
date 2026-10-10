// SPDX-License-Identifier: GPL-3.0-or-later
#include "IndicatorSdFs.h"

#include <Arduino.h>
#include <FSImpl.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include <memory>
#include <new>

#include "IndicatorSdCore.h"
#include "Rp2040Uart.h"

namespace {

// The UI thread's limits: worst case about 20 ms for the lock, 2 x 120 ms for
// a reply that does not come (SdCore asks once more), or 2 x 100 ms of BUSY.
constexpr uint32_t kUiLockWaitMs = 20;
constexpr int kUiBusyRetries = 2;
constexpr uint32_t kUiReplyTimeoutMs = 120;
// After a passing failure the UI thread's calls fail at once for this long, so
// one render pass does not pay the wait above for every path it tries. It
// doubles with each failure in a row (a long BUSY scan, a fetch holding the
// lock), up to 16 times, and resets after a call that went through.
constexpr uint32_t kUiFailFastMs = 300;
constexpr int kUiFailFastMaxShift = 4;
// Other tasks wait for the lock this long before the call fails.
constexpr uint32_t kTaskLockWaitMs = 5000;

constexpr uint32_t kPollMs = 5000;
// While the card mounts (BUSY, not yet present), or after one lost reply.
constexpr uint32_t kPollSoonMs = 1000;
// After more lost replies in a row (a bridge that stopped answering): 5 s,
// then this, so a dead link costs little UI time.
constexpr uint32_t kPollLostMaxMs = 30000;
// The poll found the lock taken.
constexpr uint32_t kPollLockRetryMs = 250;
constexpr uint32_t kStatsMs = 60000;

Rp2040Uart s_link;
indicator::SdCore* s_core = nullptr;
TaskHandle_t s_ui_task = nullptr;
bool s_bridge = false;

// Card state from the last card info; written by the poll on the UI thread.
volatile bool s_card = false;
uint64_t s_card_size = 0;
// The boot log said busy: log the first answer that settles it. Only once, as an
// empty slot reads busy again on every mount retry.
bool s_boot_busy = false;
uint32_t s_hellos = 0;
uint32_t s_next_poll_ms = 0;
int s_poll_lost = 0;  // card-info requests in a row without an answer
volatile bool s_poll_soon = false;
bool s_change = false;

// The UI thread's fail-fast window and its pending retry; UI thread only.
bool s_ui_fail_fast = false;
uint32_t s_ui_fail_until_ms = 0;
int s_ui_fail_streak = 0;
bool s_ui_retry = false;

// Tile timing on the UI thread; UI thread only.
uint32_t s_ui_ms = 0;
uint32_t s_ui_tiles = 0;

// The open write's number: a File whose write was abandoned by a newer
// writeBegin must not append to or end that newer one. Under the SD mutex.
uint32_t s_write_gen = 0;

SemaphoreHandle_t sdMutex() {
  static StaticSemaphore_t storage;
  static const SemaphoreHandle_t handle = xSemaphoreCreateRecursiveMutexStatic(&storage);
  return handle;
}

bool onUiTask() { return s_ui_task && xTaskGetCurrentTaskHandle() == s_ui_task; }

bool uiFailFast(uint32_t now) {
  if (s_ui_fail_fast && (int32_t)(now - s_ui_fail_until_ms) >= 0) s_ui_fail_fast = false;
  return s_ui_fail_fast;
}

// SdCore's link. ui is set for the outermost operation of the UI thread.
struct LinkRequester : indicator::Requester {
  bool ui = false;

  uint32_t limit(uint32_t timeout_ms) const {
    return ui && timeout_ms > kUiReplyTimeoutMs ? kUiReplyTimeoutMs : timeout_ms;
  }
  bool call(const rp2040::Msg& req, rp2040::Msg& resp, uint32_t timeout_ms) override {
    Rp2040Uart::Lock link;
    return s_link.call(req, resp, limit(timeout_ms));
  }
  int callMany(const rp2040::Msg* reqs, rp2040::Msg* resps, int n, int window) override {
    Rp2040Uart::Lock link;
    return s_link.callMany(reqs, resps, n, window, limit(rp2040::kReplyTimeoutMs));
  }
  void sleepMs(uint32_t ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }
  uint32_t nowMs() override { return millis(); }
};

LinkRequester s_req;
int s_depth = 0;

// One SdCore operation: holds the SD mutex, and the link's lock inside it, from
// the first request to the last copy out of the link's buffers. Every user of
// the link goes through here, so another task's request can never land in
// between. ok() is false when the lock was not had in time (or, on the UI
// thread, during its fail-fast window); the operation then fails. timed adds
// the UI thread's time in here to the tile timing.
class SdOp {
public:
  explicit SdOp(bool timed = true) : ui_(onUiTask()), timed_(timed), start_(millis()) {
    if (!s_core) return;
    if (ui_) {
      if (uiFailFast(start_)) return;
      if (xSemaphoreTakeRecursive(sdMutex(), pdMS_TO_TICKS(kUiLockWaitMs)) != pdTRUE) {
        missed_ = true;
        return;
      }
    } else if (xSemaphoreTakeRecursive(sdMutex(), pdMS_TO_TICKS(kTaskLockWaitMs)) != pdTRUE) {
      return;
    }
    held_ = true;
    if (s_depth++ == 0) {
      s_req.ui = ui_;
      s_core->setBusyRetries(ui_ ? kUiBusyRetries : indicator::kSdBusyRetries);
    }
    new (link_) Rp2040Uart::Lock;
  }

  ~SdOp() {
    if (ui_ && timed_) s_ui_ms += millis() - start_;
    if (!held_) return;
    if (ui_ && timed_ && !failed_) s_ui_fail_streak = 0;
    reinterpret_cast<Rp2040Uart::Lock*>(link_)->~Lock();
    if (--s_depth == 0) {
      s_req.ui = false;
      s_core->setBusyRetries(indicator::kSdBusyRetries);
    }
    xSemaphoreGiveRecursive(sdMutex());
  }

  SdOp(const SdOp&) = delete;
  SdOp& operator=(const SdOp&) = delete;

  bool ok() const { return held_; }

  // A read on the UI thread failed for a reason that passes (the lock was
  // taken, the card was busy, a reply was lost): fail fast for a while, then
  // let the map retry its missing tiles.
  void passingFailure() {
    if (!ui_ || (!held_ && !missed_)) return;
    failed_ = true;
    int shift = s_ui_fail_streak < kUiFailFastMaxShift ? s_ui_fail_streak : kUiFailFastMaxShift;
    ++s_ui_fail_streak;
    s_ui_fail_fast = true;
    s_ui_fail_until_ms = millis() + (kUiFailFastMs << shift);
    s_ui_retry = true;
  }

private:
  bool ui_;
  bool timed_;
  bool held_ = false;
  bool missed_ = false;
  bool failed_ = false;
  uint32_t start_;
  alignas(Rp2040Uart::Lock) unsigned char link_[sizeof(Rp2040Uart::Lock)];
};

// "card=present|none|busy|?": busy while the RP2040 is still mounting a card,
// ? when the card-info request got no answer. A size or free space that is not
// known prints as ?.
void logCard(const rp2040::SdInfo& info, bool got) {
  constexpr uint64_t kMB = 1024ull * 1024ull;
  const char* card = !got ? "?" : info.present ? "present" : info.busy ? "busy" : "none";
  char size_mb[24];
  char free_mb[24];
  strcpy(size_mb, "?");
  strcpy(free_mb, "?");
  if (got && (info.present || !info.busy))
    snprintf(size_mb, sizeof size_mb, "%lu", (unsigned long)(info.card_size / kMB));
  if (got && info.present && info.stats_valid)
    snprintf(free_mb, sizeof free_mb, "%lu", (unsigned long)(info.free / kMB));
  Serial.printf("[indicator] sd: card=%s size=%s free=%s\n", card, size_mb, free_mb);
}

// When to ask for the card info next: soon while a card mounts, backing off
// while the bridge does not answer.
uint32_t nextPollDelay(bool got, const rp2040::SdInfo& info) {
  if (!got) {
    ++s_poll_lost;
    return s_poll_lost <= 1 ? kPollSoonMs : s_poll_lost == 2 ? kPollMs : kPollLostMaxMs;
  }
  s_poll_lost = 0;
  return info.busy && !info.present ? kPollSoonMs : kPollMs;
}

}  // namespace

// ---- File --------------------------------------------------------------- //

class IndicatorSdFileImpl : public fs::FileImpl {
public:
  // A file open for reading, size bytes long.
  IndicatorSdFileImpl(const char* path, uint64_t size)
      : writer_(false), size_(size) {
    setPath(path);
  }
  // A file open for writing: the write numbered gen.
  IndicatorSdFileImpl(const char* path, uint32_t gen) : writer_(true), gen_(gen) { setPath(path); }

  ~IndicatorSdFileImpl() override { close(); }

  size_t write(const uint8_t* buf, size_t n) override {
    if (!open_ || !writer_ || failed_ || n == 0) return 0;
    SdOp op;
    if (!op.ok() || s_write_gen != gen_ || !s_core->writeAppend(buf, n)) {
      failed_ = true;
      return 0;
    }
    pos_ += n;
    return n;
  }

  size_t read(uint8_t* buf, size_t n) override {
    if (!open_ || writer_ || n == 0 || pos_ >= size_) return 0;
    SdOp op;
    if (!op.ok()) {
      op.passingFailure();
      return 0;
    }
    int got = s_core->readAt(path_, pos_, buf, n);
    if (got < 0) {
      op.passingFailure();
      return 0;
    }
    pos_ += (uint64_t)got;
    if (got > 0) read_any_ = true;
    return (size_t)got;
  }

  void flush() override {}

  bool seek(uint32_t pos, fs::SeekMode mode) override {
    if (!open_ || writer_) return false;
    uint64_t base = mode == fs::SeekCur ? pos_ : mode == fs::SeekEnd ? size_ : 0;
    uint64_t to = base + pos;
    if (to > size_) return false;
    pos_ = to;
    return true;
  }

  size_t position() const override { return (size_t)pos_; }
  size_t size() const override { return (size_t)(writer_ ? pos_ : size_); }
  bool setBufferSize(size_t) override { return false; }

  // A write is committed here: the last buffered bytes go out, and a write that
  // failed anywhere removes the partial file. The File is invalid afterwards.
  void close() override {
    if (!open_) return;
    open_ = false;
    if (writer_) {
      SdOp op;
      // A write() that failed before it reached the core left a gap: end the
      // write without committing it, which deletes the partial file.
      bool ok = op.ok() && s_write_gen == gen_ && s_core->writeEnd(!failed_);
      if (!ok) {
        failed_ = true;
        Serial.printf("[indicator] sd: write of %s failed\n", path_);
      }
    } else if (read_any_ && onUiTask()) {
      ++s_ui_tiles;
    }
  }

  time_t getLastWrite() override { return 0; }
  const char* path() const override { return path_; }
  const char* name() const override {
    const char* slash = strrchr(path_, '/');
    return slash ? slash + 1 : path_;
  }

  // No directories.
  boolean isDirectory(void) override { return false; }
  fs::FileImplPtr openNextFile(const char*) override { return fs::FileImplPtr(); }
  boolean seekDir(long) override { return false; }
  String getNextFileName(void) override { return String(); }
  String getNextFileName(bool* isDir) override {
    if (isDir) *isDir = false;
    return String();
  }
  void rewindDirectory(void) override {}

  operator bool() override { return open_ && !failed_; }

private:
  void setPath(const char* path) {
    strncpy(path_, path, sizeof path_ - 1);
    path_[sizeof path_ - 1] = '\0';
  }

  bool writer_;
  bool open_ = true;
  bool failed_ = false;
  bool read_any_ = false;
  uint32_t gen_ = 0;
  uint64_t pos_ = 0;
  uint64_t size_ = 0;
  char path_[rp2040::kMaxPath + 1];
};

// ---- File system -------------------------------------------------------- //

class IndicatorSdFsImpl : public fs::FSImpl {
public:
  fs::FileImplPtr open(const char* path, const char* mode, const bool create) override {
    (void)create;
    if (!s_core || !path || !mode) return fs::FileImplPtr();
    if (strcmp(mode, FILE_READ) == 0) {
      SdOp op;
      if (!op.ok()) {
        op.passingFailure();
        return fs::FileImplPtr();
      }
      uint64_t size = 0;
      int kind = s_core->stat(path, size);
      if (kind < 0) op.passingFailure();
      if (kind != 1) return fs::FileImplPtr();
      return std::make_shared<IndicatorSdFileImpl>(path, size);
    }
    if (strcmp(mode, FILE_WRITE) == 0) {
      SdOp op;
      if (!op.ok() || !s_core->writeBegin(path)) return fs::FileImplPtr();
      return std::make_shared<IndicatorSdFileImpl>(path, ++s_write_gen);
    }
    return fs::FileImplPtr();  // "a", "r+", "w+": no positioned writes on the bridge
  }

  bool exists(const char* path) override {
    if (!s_core || !path) return false;
    SdOp op;
    if (!op.ok()) {
      op.passingFailure();
      return false;
    }
    uint64_t size = 0;
    int kind = s_core->probe(path, size);  // a one-byte GET: no prefetch
    if (kind < 0) op.passingFailure();
    return kind > 0;
  }

  bool rename(const char*, const char*) override { return false; }

  bool remove(const char* path) override {
    if (!s_core || !path) return false;
    SdOp op;
    return op.ok() && s_core->remove(path);
  }

  bool mkdir(const char* path) override {
    if (!s_core || !path) return false;
    SdOp op;
    return op.ok() && s_core->mkdir(path);
  }

  bool rmdir(const char*) override { return false; }
};

IndicatorSdFs::IndicatorSdFs() : fs::FS(fs::FSImplPtr(new IndicatorSdFsImpl())) {}

IndicatorSdFs indicatorSd;

// ---- Board side --------------------------------------------------------- //

bool indicatorSdBegin() {
  if (s_core) return s_bridge;
  if (!s_link.begin()) return false;  // logs why

  // SdCore (about 36 KB) and its read-ahead window (32 KB) live in PSRAM.
  void* window = heap_caps_malloc(indicator::kSdReadAheadBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  void* mem = heap_caps_malloc(sizeof(indicator::SdCore), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!window || !mem) {
    heap_caps_free(window);
    heap_caps_free(mem);
    Serial.println("[indicator] sd: off (out of memory)");
    return false;
  }
  s_core = new (mem) indicator::SdCore(s_req, static_cast<uint8_t*>(window));
  s_bridge = true;

  // No UI task yet: this one waits with the full limits.
  rp2040::SdInfo info;
  bool got;
  {
    SdOp op;
    got = op.ok() && s_core->pollInfo(info);
    s_hellos = s_link.helloCount();
  }
  if (got) {
    s_card = info.present;
    s_card_size = info.card_size;
    s_boot_busy = info.busy && !info.present;
  }
  logCard(info, got);
  s_next_poll_ms = millis() + nextPollDelay(got, info);
  s_ui_task = xTaskGetCurrentTaskHandle();
  return true;
}

bool indicatorSdAvailable() { return s_bridge; }

bool indicatorSdReady() { return s_bridge && s_card; }

bool indicatorSdWritable() { return s_bridge && s_core->writesSupported(); }

void indicatorSdPoll(uint32_t now_ms) {
  if (!s_bridge) return;
  // An early poll is not worth it once the bridge stopped answering.
  const bool soon = s_poll_soon && s_poll_lost < 2;
  if (!soon && (int32_t)(now_ms - s_next_poll_ms) < 0) return;
  SdOp op(false);
  s_poll_soon = false;  // a lock miss below waits out its own retry time
  if (!op.ok()) {
    s_next_poll_ms = now_ms + kPollLockRetryMs;
    return;
  }
  rp2040::SdInfo info;
  bool got = s_core->pollInfo(info);
  bool changed = false;
  uint32_t hellos = s_link.helloCount();
  if (hellos != s_hellos) {
    // The RP2040 restarted: whatever the caches hold is from before.
    s_hellos = hellos;
    s_core->invalidate();
    changed = true;
  }
  if (got) {
    if (info.present != s_card || (info.present && info.card_size != s_card_size)) changed = true;
    s_card = info.present;
    s_card_size = info.card_size;
  }
  // No answer: keep what was known.
  s_next_poll_ms = now_ms + nextPollDelay(got, info);
  const bool settled = got && s_boot_busy && !(info.busy && !info.present);
  if (settled) s_boot_busy = false;
  if (changed) {
    s_change = true;
    if (got) logCard(info, got);
  } else if (settled) {
    logCard(info, got);
  }
}

bool indicatorSdTakeChange() {
  bool c = s_change;
  s_change = false;
  return c;
}

bool indicatorSdTakeRetry() {
  if (!s_ui_retry || uiFailFast(millis())) return false;
  s_ui_retry = false;
  return true;
}

bool indicatorSdReadBlocked() { return uiFailFast(millis()); }

void indicatorSdPollSoon() { s_poll_soon = true; }

void indicatorSdLogStats(uint32_t now_ms) {
  static uint32_t s_window_end = 0;
  static uint32_t s_last_call = 0;
  // A new window after a pause: the map was closed or off the card.
  if (s_window_end == 0 || now_ms - s_last_call > 5000) {
    s_window_end = now_ms + kStatsMs;
    s_ui_ms = 0;
    s_ui_tiles = 0;
  }
  s_last_call = now_ms;
  if ((int32_t)(now_ms - s_window_end) < 0) return;
  s_window_end = now_ms + kStatsMs;
  Serial.printf("[indicator] sd: tiles=%lu avg=%lums/tile\n", (unsigned long)s_ui_tiles,
                (unsigned long)(s_ui_tiles ? s_ui_ms / s_ui_tiles : 0));
  s_ui_ms = 0;
  s_ui_tiles = 0;
}
