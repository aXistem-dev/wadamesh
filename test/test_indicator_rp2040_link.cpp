// SPDX-License-Identifier: GPL-3.0-or-later
//
// Host test for the SenseCAP Indicator RP2040 bridge link (Rp2040Link.h): the
// framer's resync and stall handling, and LinkCore's request/reply pairing over
// a scripted fake transport with a fake clock. The message codec is a stand-in
// defined here, so the test needs no protobuf runtime.
//
//   c++ -std=c++17 -Wall -Wextra -I src -I variants/sensecap_indicator test/test_indicator_rp2040_link.cpp -o /tmp/ind_link && /tmp/ind_link

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <functional>
#include <vector>

#include "Rp2040Link.h"

using namespace rp2040;

using Bytes = std::vector<uint8_t>;

// --- stand-in codec ---------------------------------------------------------- //
//
// A payload is 0xA5, a 4-byte index into g_msgs, then the file data. Decoded
// file data points into the payload, as with the real codec it points into a
// buffer the next decode reuses: the link must copy it out.

static std::vector<Msg> g_msgs;

size_t rp2040::encode(const Msg& m, uint8_t* out, size_t cap) {
  const size_t data_len = m.kind == Kind::File ? m.file.data_len : 0;
  if (5 + data_len > cap) return 0;
  const uint32_t i = (uint32_t)g_msgs.size();
  g_msgs.push_back(m);
  out[0] = 0xA5;
  memcpy(out + 1, &i, 4);
  if (data_len) memcpy(out + 5, m.file.data, data_len);
  return 5 + data_len;
}

bool rp2040::decode(const uint8_t* in, size_t n, Msg& out) {
  uint32_t i;
  if (n < 5 || in[0] != 0xA5) return false;
  memcpy(&i, in + 1, 4);
  if (i >= g_msgs.size()) return false;
  out = g_msgs[i];
  const size_t data_len = out.kind == Kind::File ? out.file.data_len : 0;
  if (n != 5 + data_len) return false;
  out.file.data = data_len ? in + 5 : nullptr;
  return true;
}

// --- helpers ----------------------------------------------------------------- //

static Bytes wrap(const uint8_t* payload, size_t n) {
  Bytes f = {0x94, 0xC3, (uint8_t)(n >> 8), (uint8_t)(n & 0xFF)};
  f.insert(f.end(), payload, payload + n);
  return f;
}

static Bytes frameOf(const Msg& m) {
  static uint8_t buf[kMaxPayload];
  size_t n = encode(m, buf, sizeof buf);
  assert(n > 0);
  return wrap(buf, n);
}

static void feed(Framer& f, const Bytes& b) {
  size_t room = 0;
  uint8_t* dst = f.space(room);
  assert(b.size() <= room);
  memcpy(dst, b.data(), b.size());
  f.commit(b.size());
}

static void expectFrame(Framer& f, const uint8_t* want, size_t want_len) {
  const uint8_t* p = nullptr;
  size_t n = 0;
  assert(f.next(p, n));
  assert(n == want_len && memcmp(p, want, n) == 0);
  assert(!f.next(p, n));
}

static Msg make(Kind kind, uint32_t id) {
  Msg m{};
  m.kind = kind;
  m.id = id;
  return m;
}

static Msg pong(uint32_t id, uint32_t version) {
  Msg m = make(Kind::Pong, id);
  m.version = version;
  return m;
}

static Msg getReq(const char* path, uint64_t offset) {
  Msg m = make(Kind::File, 0);
  m.file.op = Op::Get;
  strcpy(m.file.path, path);
  m.file.offset = offset;
  return m;
}

// A GET reply whose data is `len` bytes of `fill`.
static Msg getReply(uint32_t id, const Msg& req, uint8_t fill, uint16_t len) {
  static uint8_t data[kMaxChunk];
  memset(data, fill, len);
  Msg m = make(Kind::File, id);
  m.file = req.file;
  m.file.status = Status::Ok;
  m.file.data = data;
  m.file.data_len = len;
  return m;
}

// RX bytes come from a queue the test (or the responder) fills; TX frames are
// decoded and handed to the responder, which plays the bridge. read() with
// nothing queued advances the clock by its whole timeout, as a real blocking
// read would.
class FakeTransport : public Transport {
public:
  std::deque<uint8_t> rx;
  Bytes tx;
  std::vector<Msg> requests;
  std::function<void(const Msg&)> responder;
  uint32_t now = 1000;
  size_t maxRead = 512;  // bytes per read(), so frames arrive in pieces

  void queue(const Bytes& b) { rx.insert(rx.end(), b.begin(), b.end()); }
  void queue(const Msg& m) { queue(frameOf(m)); }

  size_t write(const uint8_t* data, size_t n) override {
    tx.insert(tx.end(), data, data + n);
    while (tx.size() >= 4) {
      assert(tx[0] == 0x94 && tx[1] == 0xC3);
      size_t len = (size_t)tx[2] << 8 | tx[3];
      if (tx.size() < 4 + len) break;
      Msg m;
      assert(decode(tx.data() + 4, len, m));
      tx.erase(tx.begin(), tx.begin() + 4 + len);
      requests.push_back(m);
      if (responder) responder(m);
    }
    return n;
  }

  size_t read(uint8_t* buf, size_t n, uint32_t timeout_ms) override {
    if (rx.empty()) {
      now += timeout_ms;
      return 0;
    }
    size_t k = 0;
    while (k < n && k < maxRead && !rx.empty()) {
      buf[k++] = rx.front();
      rx.pop_front();
    }
    return k;
  }

  uint32_t nowMs() override { return now; }
};

// --- framer ------------------------------------------------------------------ //

// Garbage, a stray magic byte and a header claiming more than kMaxPayload cost
// only themselves; a maximum frame fits the buffer exactly.
static void framer_resyncs_past_garbage_and_false_headers() {
  const uint8_t want[] = {0x58, 0x07, 0x5A, 0x02};
  {
    Framer f;
    feed(f, {0x00, 0x94, 0x12, 0xC3, 0x94, 0x94, 0xFF, 0x13, 0x37, 0x94});
    feed(f, wrap(want, sizeof want));
    expectFrame(f, want, sizeof want);
  }
  {
    Framer f;
    feed(f, {0x94, 0xC3, 0x13, 0x88, 0x01, 0x02, 0x03});  // 5000 bytes claimed
    feed(f, wrap(want, sizeof want));
    expectFrame(f, want, sizeof want);
  }
  {
    Framer f;
    static uint8_t big[kMaxPayload];
    for (size_t i = 0; i < sizeof big; ++i) big[i] = (uint8_t)i;
    feed(f, wrap(big, sizeof big));
    expectFrame(f, big, sizeof big);
  }
}

// A frame arriving a byte at a time, 20 ms apart, is not stalled; a partial
// frame that gets nothing for over 30 ms is dropped, and a real frame hidden
// inside a false header's claimed length is found once the bytes stop.
static void framer_split_and_stalled_frames() {
  const uint8_t want[] = {0x58, 0x0C, 0x5A, 0x02, 0x22, 0x23};
  const Bytes good = wrap(want, sizeof want);
  const uint8_t* p = nullptr;
  size_t n = 0;
  {
    Framer f;
    uint32_t t = 7000;
    for (size_t i = 0; i < good.size(); ++i) {
      feed(f, Bytes{good[i]});
      f.tick(t += 20);
      assert(f.next(p, n) == (i + 1 == good.size()));
    }
    assert(n == sizeof want && memcmp(p, want, n) == 0);
  }
  {
    Framer f;
    uint8_t stale[64];
    memset(stale, 0x11, sizeof stale);
    Bytes half = wrap(stale, sizeof stale);
    half.resize(half.size() / 2);
    feed(f, half);
    f.tick(5000);
    assert(!f.next(p, n));
    f.tick(5031);
    feed(f, good);
    expectFrame(f, want, sizeof want);
  }
  {
    Framer f;
    feed(f, {0x94, 0xC3, 0x01, 0x00});  // claims 256 bytes
    feed(f, good);
    f.tick(100);
    assert(!f.next(p, n));
    f.tick(131);
    expectFrame(f, want, sizeof want);
  }
}

// --- link core --------------------------------------------------------------- //

static void ping_requires_version_2() {
  FakeTransport t;
  uint32_t reply = 1;
  t.responder = [&](const Msg& m) {
    assert(m.kind == Kind::Ping && m.version == kVersion && m.id != 0);
    t.queue(pong(m.id, reply));
  };
  LinkCore link(t);
  uint32_t v = 0;
  assert(!link.ping(v));
  assert(v == 1);
  reply = 2;
  assert(link.ping(v));
  assert(v == 2);
}

// A reply is the frame with the request's id. A late reply for an earlier id,
// a nack for another request, nmea and the bridge's boot hello (a ping with id
// 0, counted) are passed over; a 4 KB reply read 7 bytes at a time is
// reassembled and its data copied out of the codec's buffer.
static void reply_pairs_by_id() {
  FakeTransport t;
  t.maxRead = 7;
  t.responder = [&](const Msg& m) {
    t.queue(getReply(m.id - 1, m, 0xEE, 16));
    t.queue(make(Kind::Nack, m.id + 7));
    t.queue(make(Kind::Other, 0));
    Msg hello = make(Kind::Ping, 0);
    hello.version = 2;
    t.queue(hello);
    t.queue(getReply(m.id, m, 0x5A, (uint16_t)kMaxChunk));
    t.queue(make(Kind::Other, 0));
  };
  LinkCore link(t);
  link.setNextId(10);  // so the late reply carries a real id (9)
  Msg req = getReq("/maps/a.png", 4096);
  Msg resp;
  assert(link.call(req, resp));
  assert(t.requests.size() == 1);
  assert(resp.kind == Kind::File && resp.id == t.requests[0].id);
  assert(resp.file.offset == 4096 && resp.file.data_len == kMaxChunk);
  for (size_t b = 0; b < kMaxChunk; ++b) assert(resp.file.data[b] == 0x5A);
  assert(link.helloCount() == 1);
}

static void call_times_out_after_500ms() {
  FakeTransport t;
  LinkCore link(t);
  Msg req = make(Kind::GetSdInfo, 0);
  Msg resp;
  uint32_t start = t.now;
  assert(!link.call(req, resp));
  assert(t.now - start >= 500 && t.now - start < 600);
  assert(resp.kind == Kind::None);
}

// A nack with the request's id, or with id 0 (a frame the bridge could not
// decode), fails at once; in a batch it ends the batch at that request.
static void nack_fails_fast() {
  for (uint32_t which : {1u, 0u}) {
    FakeTransport t;
    t.responder = [&](const Msg& m) { t.queue(make(Kind::Nack, which ? m.id : 0)); };
    LinkCore link(t);
    Msg req = getReq("/x", 0);
    req.file.op = Op::Mkdir;
    Msg resp;
    uint32_t start = t.now;
    assert(!link.call(req, resp));
    assert(resp.kind == Kind::Nack);
    assert(t.now - start < 100);
  }
  FakeTransport t;
  int seen = 0;
  t.responder = [&](const Msg& m) {
    if (seen++ == 1)
      t.queue(make(Kind::Nack, m.id));
    else
      t.queue(getReply(m.id, m, 0x44, 4));
  };
  LinkCore link(t);
  Msg reqs[3] = {getReq("/n", 0), getReq("/n", 4), getReq("/n", 8)};
  Msg resps[3];
  uint32_t start = t.now;
  assert(link.callMany(reqs, resps, 3, 3) == 1);
  assert(resps[1].kind == Kind::Nack);
  assert(t.now - start < 100);
}

static void ids_count_up_and_wrap_to_one() {
  FakeTransport t;
  t.responder = [&](const Msg& m) { t.queue(pong(m.id, 2)); };
  LinkCore link(t);
  uint32_t v = 0;
  for (int i = 0; i < 3; ++i) assert(link.ping(v));
  link.setNextId(0x7FFFFFFF);
  assert(link.ping(v) && link.ping(v));
  const uint32_t want[] = {1, 2, 3, 0x7FFFFFFF, 1};
  assert(t.requests.size() == 5);
  for (size_t i = 0; i < 5; ++i) assert(t.requests[i].id == want[i]);
}

// Three GETs in flight answered out of order: each response pairs with its
// request by id and keeps its own data. With window 2 only two requests are
// on the wire: the third goes out after the first reply.
static void callmany_pairs_out_of_order_within_window() {
  {
    FakeTransport silent;
    LinkCore link(silent);
    Msg reqs[5];
    for (int i = 0; i < 5; ++i) reqs[i] = getReq("/w", (uint64_t)i * 8);
    Msg resps[5];
    assert(link.callMany(reqs, resps, 5, 2) == 0);
    assert(silent.requests.size() == 2);
  }
  {
    FakeTransport t;
    std::vector<Msg> held;
    t.responder = [&](const Msg& m) {
      held.push_back(m);
      if (held.size() < 3) return;
      for (int k : {2, 0, 1})
        t.queue(getReply(held[k].id, held[k], (uint8_t)(0xA0 + k), (uint16_t)kMaxChunk));
      held.clear();
    };
    LinkCore link(t);
    Msg reqs[3] = {getReq("/t", 0), getReq("/t", kMaxChunk), getReq("/t", 2 * kMaxChunk)};
    Msg resps[3];
    assert(link.callMany(reqs, resps, 3, 3) == 3);
    for (int i = 0; i < 3; ++i) {
      assert(resps[i].id == t.requests[i].id);
      assert(resps[i].file.offset == (uint64_t)i * kMaxChunk);
      for (size_t b = 0; b < kMaxChunk; ++b) assert(resps[i].file.data[b] == 0xA0 + i);
    }
  }
  FakeTransport t;
  std::vector<Msg> held;
  size_t maxHeld = 0;
  t.responder = [&](const Msg& m) {
    held.push_back(m);
    if (held.size() > maxHeld) maxHeld = held.size();
    if (held.size() == 2) {
      t.queue(getReply(held.front().id, held.front(), 0x01, 8));
      held.erase(held.begin());
    }
  };
  LinkCore link(t);
  Msg reqs[5];
  for (int i = 0; i < 5; ++i) reqs[i] = getReq("/w", (uint64_t)i * 8);
  Msg resps[5];
  assert(link.callMany(reqs, resps, 5, 2) == 4);  // the last one stays unanswered
  assert(maxHeld == 2 && t.requests.size() == 5);
  for (int i = 0; i < 4; ++i) assert(resps[i].id == t.requests[i].id);
}

// A lost reply: callMany reports the replies received before it, in order.
static void callmany_lost_reply_returns_prefix() {
  FakeTransport t;
  int seen = 0;
  t.responder = [&](const Msg& m) {
    if (seen++ != 1) t.queue(getReply(m.id, m, 0x33, 4));
  };
  LinkCore link(t);
  Msg reqs[3] = {getReq("/l", 0), getReq("/l", 4), getReq("/l", 8)};
  Msg resps[3];
  uint32_t start = t.now;
  assert(link.callMany(reqs, resps, 3, 3) == 1);
  assert(resps[0].id == t.requests[0].id);
  assert(resps[1].kind == Kind::None);
  assert(t.now - start >= 500 && t.now - start < 600);
}

// A false header whose payload does not decode: the link looks again from
// just after its magic. Here the real frame starts inside the false header,
// its first byte being the false length's low byte.
static void undecodable_frame_resyncs_inside() {
  FakeTransport t;
  t.responder = [&](const Msg& m) {
    Bytes real = frameOf(pong(m.id, 2));
    Bytes f = {0x94, 0xC3, 0x00};
    f.insert(f.end(), real.begin(), real.end());
    f.resize(4 + 0x94, 0x00);  // the false header claims 0x94 bytes
    t.queue(f);
  };
  LinkCore link(t);
  uint32_t v = 0;
  assert(link.ping(v) && v == 2);
}

// post() sends one frame with a fresh id and returns without reading or
// waiting, since the bridge never answers a beep; the next call still pairs.
static void post_sends_without_waiting() {
  FakeTransport t;
  LinkCore link(t);
  link.setNextId(5);
  Msg beep = make(Kind::Beep, 0);
  beep.beep_ms = 80;
  uint32_t start = t.now;
  assert(link.post(beep));
  assert(t.now == start);
  assert(t.requests.size() == 1);
  assert(t.requests[0].kind == Kind::Beep && t.requests[0].beep_ms == 80 && t.requests[0].id == 5);
  t.responder = [&](const Msg& m) { t.queue(pong(m.id, kVersion)); };
  uint32_t v = 0;
  assert(link.ping(v) && v == kVersion);
  assert(t.requests[1].id == 6);
}

int main() {
  framer_resyncs_past_garbage_and_false_headers();
  framer_split_and_stalled_frames();
  ping_requires_version_2();
  reply_pairs_by_id();
  call_times_out_after_500ms();
  nack_fails_fast();
  ids_count_up_and_wrap_to_one();
  callmany_pairs_out_of_order_within_window();
  callmany_lost_reply_returns_prefix();
  undecodable_frame_resyncs_inside();
  post_sends_without_waiting();
  printf("indicator rp2040 link: all tests passed\n");
  return 0;
}
