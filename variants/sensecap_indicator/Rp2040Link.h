// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Request/reply link to the SenseCAP Indicator's RP2040 bridge firmware. Frames
// are 0x94 0xC3, a big-endian u16 payload length, then an InterdeviceMessage
// (Rp2040Msg.h). The bridge answers requests strictly in order and echoes each
// request's id; replies can still be lost, so they are paired by id, never by
// position. Hardware-free: the UART sits behind Transport, so the host test
// drives this file as is. Not thread-safe; Rp2040Uart serialises callers.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Rp2040Msg.h"

namespace rp2040 {

constexpr uint8_t kMagic0 = 0x94;
constexpr uint8_t kMagic1 = 0xC3;
constexpr size_t kHeaderSize = 4;
constexpr size_t kMaxFrame = kHeaderSize + kMaxPayload;
// A whole frame takes about 23 ms at 2 Mbaud. A frame still incomplete after
// this long without a new byte had a false or damaged header.
constexpr uint32_t kStallMs = 30;
constexpr uint32_t kReplyTimeoutMs = 500;
// Most requests one callMany keeps track of, and so most file chunks it holds.
constexpr int kMaxBatch = 8;
// Request ids run 1..kMaxId; 0 marks unsolicited frames.
constexpr uint32_t kMaxId = 0x7FFFFFFF;

struct Transport {
  // Writes all n bytes, or returns how many went out.
  virtual size_t write(const uint8_t* data, size_t n) = 0;
  // Up to n bytes: what is there now, else waits up to timeout_ms for the
  // first and returns 0 when none come.
  virtual size_t read(uint8_t* buf, size_t n, uint32_t timeout_ms) = 0;
  virtual uint32_t nowMs() = 0;
  virtual ~Transport() = default;
};

// Cuts the received byte stream into frame payloads. Bad bytes cost only
// themselves: the framer resyncs on the next magic, a length over kMaxPayload
// marks a false header, and a partial frame that gets no byte for kStallMs is
// dropped (tick) without losing a complete frame queued behind its header.
class Framer {
public:
  // Space for new bytes, after moving the buffered ones to the front: write at
  // most `room` bytes there, then commit them. Room is never 0 while next()
  // returns false, since one frame always fits the buffer.
  uint8_t* space(size_t& room) {
    if (start_ > 0) {
      memmove(buf_, buf_ + start_, end_ - start_);
      end_ -= start_;
      start_ = 0;
    }
    last_ = kNone;
    room = sizeof buf_ - end_;
    return buf_ + end_;
  }

  void commit(size_t n) {
    if (n == 0) return;
    end_ += n;
    fresh_ = true;
  }

  // The next complete frame's payload. It points into the framer and is valid
  // until the next space() or tick().
  bool next(const uint8_t*& payload, size_t& len) {
    for (;;) {
      if (!seekMagic()) return false;
      if (end_ - start_ < kHeaderSize) return false;
      size_t n = claimed();
      if (n > kMaxPayload) {
        ++start_;  // not a real header: look for the next magic
        continue;
      }
      if (end_ - start_ < kHeaderSize + n) return false;
      payload = buf_ + start_ + kHeaderSize;
      len = n;
      last_ = start_;
      start_ += kHeaderSize + n;
      return true;
    }
  }

  // The frame next() just returned did not decode: it had a false header, so
  // look for a frame from just after its magic.
  void reject() {
    if (last_ == kNone) return;
    start_ = last_ + 1;
    last_ = kNone;
  }

  // Call with the time now, after each read. When no byte has come for longer
  // than kStallMs, every incomplete frame ahead of the next complete one goes.
  void tick(uint32_t now_ms) {
    last_ = kNone;
    if (fresh_) {
      fresh_ = false;
      last_rx_ms_ = now_ms;
      return;
    }
    if (now_ms - last_rx_ms_ <= kStallMs) return;
    for (;;) {
      if (!seekMagic() || end_ - start_ < kHeaderSize) {
        start_ = end_;  // a stale trailing magic byte or header goes too
        break;
      }
      size_t n = claimed();
      if (n <= kMaxPayload && end_ - start_ >= kHeaderSize + n) break;  // complete
      ++start_;
    }
    if (start_ == end_) start_ = end_ = 0;
  }

private:
  static constexpr size_t kNone = (size_t)-1;

  // Moves start_ to the next full magic and returns true, or drops all but a
  // trailing first magic byte (its partner may be on its way) and returns false.
  bool seekMagic() {
    size_t i = start_;
    while (i + 1 < end_ && !(buf_[i] == kMagic0 && buf_[i + 1] == kMagic1)) ++i;
    if (i + 1 < end_) {
      start_ = i;
      return true;
    }
    start_ = (i < end_ && buf_[i] == kMagic0) ? i : end_;
    if (start_ == end_) start_ = end_ = 0;
    return false;
  }

  size_t claimed() const { return (size_t)buf_[start_ + 2] << 8 | buf_[start_ + 3]; }

  uint8_t buf_[kMaxFrame];
  size_t start_ = 0;  // buffered bytes are buf_[start_, end_)
  size_t end_ = 0;
  size_t last_ = kNone;  // start of the frame next() last returned
  uint32_t last_rx_ms_ = 0;
  bool fresh_ = false;  // bytes committed since the last tick
};

// Requests and their replies over one Transport.
//
// Lifetime: a response's file.data points into this LinkCore, not into the
// transport's buffers, and stays valid until the next ping, call or callMany
// on it. Callers sharing a link hold its lock across the call and the reading
// of that data.
class LinkCore {
public:
  explicit LinkCore(Transport& transport) : t_(transport) {}
  LinkCore(const LinkCore&) = delete;
  LinkCore& operator=(const LinkCore&) = delete;

  // Pings with our version. version gets the peer's (0 without a reply); true
  // only for a pong in kVersion.
  bool ping(uint32_t& version) {
    Msg req;
    req.kind = Kind::Ping;
    req.version = kVersion;
    Msg resp;
    version = 0;
    if (!call(req, resp)) return false;
    if (resp.kind != Kind::Pong) return false;
    version = resp.version;
    return version == kVersion;
  }

  // Sends req (its id is replaced) and waits up to timeout_ms for the reply.
  // False on a timeout (resp.kind None) or a nack (resp.kind Nack).
  bool call(const Msg& req, Msg& resp, uint32_t timeout_ms = kReplyTimeoutMs) {
    return callMany(&req, &resp, 1, 1, timeout_ms) == 1;
  }

  // Sends reqs[0..n), at most `window` beyond the first unanswered one, and
  // pairs each reply with its request by id: resps[i] answers reqs[i]. Returns
  // how many replies came in order, so resps[0..result) are valid. The wait
  // for a reply is timeout_ms from the last one received. A nack ends the batch
  // at its request (resps there holds the nack); a nack with id 0 is for a
  // request the bridge could not decode, so it ends the batch at the first
  // unanswered one. n is capped at kMaxBatch.
  int callMany(const Msg* reqs, Msg* resps, int n, int window = 3,
               uint32_t timeout_ms = kReplyTimeoutMs) {
    if (n <= 0) return 0;
    if (n > kMaxBatch) n = kMaxBatch;
    if (window < 1) window = 1;
    uint32_t ids[kMaxBatch];
    bool got[kMaxBatch] = {};
    for (int i = 0; i < n; ++i) resps[i] = Msg{};
    int sent = 0;
    int done = 0;   // resps[0..done) answered
    int limit = n;  // lowered by a nack or a failed send
    uint32_t since = t_.nowMs();
    // Bytes only count as received once read: take in what waited while the
    // link was idle, so its first tick does not see them as a stall.
    pump(0);

    while (done < limit) {
      while (sent < limit && sent - done < window) {
        uint32_t id = takeId();
        if (!send(reqs[sent], id)) {
          limit = sent;
          break;
        }
        ids[sent++] = id;
      }

      for (;;) {
        rx_.tick(t_.nowMs());
        const uint8_t* payload;
        size_t len;
        if (!rx_.next(payload, len)) break;
        if (!decode(payload, len, in_)) {
          rx_.reject();
          continue;
        }
        if (in_.id == 0 && in_.kind == Kind::Ping) {
          ++hellos_;  // the bridge (re)started
          continue;
        }
        int k = -1;
        if (in_.id != 0) {
          for (int i = done; i < sent; ++i) {
            if (!got[i] && ids[i] == in_.id) {
              k = i;
              break;
            }
          }
        }
        if (in_.kind == Kind::Nack && (in_.id == 0 || k >= 0)) {
          int at = in_.id == 0 ? done : k;
          if (at < limit) {
            resps[at] = in_;
            limit = at;
          }
          continue;
        }
        if (k < 0) continue;  // a late reply, nmea or other unsolicited frame
        resps[k] = in_;
        if (in_.kind == Kind::File && in_.file.data_len > 0) {
          memcpy(data_[k], in_.file.data, in_.file.data_len);
          resps[k].file.data = data_[k];
        } else {
          resps[k].file.data = nullptr;
        }
        got[k] = true;
        while (done < sent && got[done]) ++done;
        since = t_.nowMs();
      }
      if (done >= limit) break;

      uint32_t waited = t_.nowMs() - since;
      if (waited >= timeout_ms) break;
      uint32_t wait = timeout_ms - waited;
      if (wait > kPollMs) wait = kPollMs;  // wake up to notice a stalled frame
      pump(wait);
    }
    return done < limit ? done : limit;
  }

  // Sends req (its id is replaced) without waiting: for a message the bridge
  // does not answer, such as Beep. False when it does not encode or go out.
  bool post(const Msg& req) { return send(req, takeId()); }

  // Unsolicited pings with id 0 seen so far: the bridge says it (re)started.
  uint32_t helloCount() const { return hellos_; }

  // The id the next request gets. Seeding it at random after a reset keeps a
  // reply the bridge still owes the previous run from matching a new request.
  void setNextId(uint32_t id) { next_id_ = (id == 0 || id > kMaxId) ? 1 : id; }

private:
  static constexpr uint32_t kPollMs = 10;

  uint32_t takeId() {
    uint32_t id = next_id_;
    next_id_ = id < kMaxId ? id + 1 : 1;
    return id;
  }

  void pump(uint32_t wait_ms) {
    size_t room = 0;
    uint8_t* dst = rx_.space(room);
    rx_.commit(t_.read(dst, room, wait_ms));
  }

  bool send(const Msg& req, uint32_t id) {
    out_ = req;
    out_.id = id;
    size_t n = encode(out_, tx_ + kHeaderSize, kMaxPayload);
    if (n == 0) return false;
    tx_[0] = kMagic0;
    tx_[1] = kMagic1;
    tx_[2] = (uint8_t)(n >> 8);
    tx_[3] = (uint8_t)(n & 0xFF);
    return t_.write(tx_, kHeaderSize + n) == kHeaderSize + n;
  }

  Transport& t_;
  Framer rx_;
  uint8_t tx_[kMaxFrame];
  uint8_t data_[kMaxBatch][kMaxChunk];  // file data of the replies of the last call
  Msg in_;
  Msg out_;
  uint32_t next_id_ = 1;
  uint32_t hellos_ = 0;
};

}  // namespace rp2040
