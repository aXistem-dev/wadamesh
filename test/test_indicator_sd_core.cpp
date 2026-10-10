// SPDX-License-Identifier: GPL-3.0-or-later
//
// Host test for SdCore (IndicatorSdCore.h), the SenseCAP Indicator's file
// access to the microSD behind the RP2040 bridge. FakeCard plays the bridge: an
// in-memory card with the bridge's file_transfer semantics (ranged GET, POST at
// 0, PUT at the current size else OFFSET_CONFLICT with the size, MKDIR with
// parents, idempotent DELETE) and switches for BUSY, NO_CARD, a pulled card,
// lost replies, nacks and the stock firmware's "unknown operation" answer.
//
//   c++ -std=c++17 -Wall -Wextra -I src -I variants/sensecap_indicator test/test_indicator_sd_core.cpp -o /tmp/ind_sd && /tmp/ind_sd

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "IndicatorSdCore.h"

using namespace rp2040;
using indicator::SdCore;

using Bytes = std::vector<uint8_t>;

static Bytes pattern(size_t n, uint8_t seed) {
  Bytes b(n);
  for (size_t i = 0; i < n; ++i) b[i] = (uint8_t)(seed + i * 7 + (i >> 9));
  return b;
}

struct FakeCard : indicator::Requester {
  std::map<std::string, Bytes> files;
  std::set<std::string> dirs{"/"};

  // Switches.
  bool noCard = false;       // every file request: NO_CARD
  bool busyForever = false;  // every file request: BUSY
  int busyNext = 0;          // the next n file requests: BUSY
  bool nackMkdir = false;    // bridge firmware without MKDIR that nacks it
  bool stockFirmware = false;  // MKDIR answered IO_ERROR "unknown operation"
  // A card pulled before the bridge noticed: a GET fails like a missing file,
  // or IO_ERROR for the file it holds open; writes fail; a DELETE finds
  // nothing there and answers OK.
  bool pulled = false;

  // One-off faults on the nth request (from 1) of an operation.
  enum class What { Drop, Nack, GarbledNack, Reply };
  struct Fault {
    Op op;
    int nth;
    What what;
    Status status;  // What::Reply: answered with this, the card left alone
  };
  std::vector<Fault> faults;

  SdInfo info;  // what get_sd_info answers

  // What the card saw.
  std::map<Op, int> count;
  std::map<Op, uint32_t> timeout_ms;  // per operation, of the last single request
  std::vector<uint32_t> putSizes;
  uint32_t lastGetLength = 0;
  int batches = 0;
  int maxGetWindow = 0;
  bool writeInBatch = false;
  uint32_t slept = 0;
  uint32_t clock = 0;

  FakeCard() {
    info.present = true;
    info.card_size = 8ull << 30;
  }

  void addFile(const std::string& path, const Bytes& b) {
    files[path] = b;
    for (size_t i = path.find('/', 1); i != std::string::npos; i = path.find('/', i + 1))
      dirs.insert(path.substr(0, i));
  }

  int n(Op op) const {
    auto it = count.find(op);
    return it == count.end() ? 0 : it->second;
  }

  bool call(const Msg& req, Msg& resp, uint32_t t) override {
    if (req.kind == Kind::File) timeout_ms[req.file.op] = t;
    poison();
    resp = Msg{};
    return serve(req, resp, 0);
  }

  int callMany(const Msg* reqs, Msg* resps, int n, int window) override {
    poison();
    ++batches;
    for (int i = 0; i < n; ++i) {
      if (reqs[i].kind == Kind::File && reqs[i].file.op != Op::Get) writeInBatch = true;
      resps[i] = Msg{};
    }
    if (n > 1) maxGetWindow = std::max(maxGetWindow, window);
    for (int i = 0; i < n; ++i) {
      if (!serve(reqs[i], resps[i], i)) {
        // What the link had already sent behind it reaches the card all the same.
        for (int j = i + 1; j < std::min(n, i + window); ++j) {
          Msg lost;
          serve(reqs[j], lost, j);
        }
        return i;
      }
    }
    return n;
  }

  void sleepMs(uint32_t ms) override {
    slept += ms;
    clock += ms;
  }
  uint32_t nowMs() override { return clock; }

private:
  // Reply data lives here until the next call, as it does in the link.
  uint8_t data_[kMaxBatch][kMaxChunk];
  std::string open_;  // the bridge's one-file read cache
  uint32_t next_id_ = 1;

  void poison() { memset(data_, 0xEE, sizeof data_); }

  Fault* fault(Op op, int nth) {
    for (auto& f : faults)
      if (f.op == op && f.nth == nth) return &f;
    return nullptr;
  }

  static std::string parentOf(const std::string& p) {
    size_t i = p.rfind('/');
    return i == 0 || i == std::string::npos ? "/" : p.substr(0, i);
  }

  bool serve(const Msg& req, Msg& resp, int slot) {
    uint32_t id = next_id_++;
    if (req.kind == Kind::GetSdInfo) {
      resp.kind = Kind::SdInfo;
      resp.id = id;
      resp.sd = info;
      if (noCard) {
        resp.sd = SdInfo{};
        resp.sd.busy = busyForever;
      }
      return true;
    }
    assert(req.kind == Kind::File);
    const FileTransfer& q = req.file;
    Fault* f = fault(q.op, ++count[q.op]);
    if (f && f->what == What::GarbledNack) {
      resp.kind = Kind::Nack;
      resp.id = 0;
      return false;
    }
    if ((f && f->what == What::Nack) || (q.op == Op::Mkdir && nackMkdir)) {
      resp.kind = Kind::Nack;
      resp.id = id;
      return false;
    }
    resp.kind = Kind::File;
    resp.id = id;
    FileTransfer& r = resp.file;
    r.op = q.op;
    strcpy(r.path, q.path);
    r.offset = q.offset;
    if (q.op == Op::Get) lastGetLength = q.length;
    if (f && f->what == What::Reply) {
      r.status = f->status;
    } else if (stockFirmware && q.op == Op::Mkdir) {
      r.status = Status::IoError;
      strcpy(r.message, "unknown operation");
    } else if (noCard) {
      r.status = Status::NoCard;
    } else if (busyForever || busyNext > 0) {
      if (busyNext > 0) --busyNext;
      r.status = Status::Busy;
    } else if (pulled) {
      if (q.op == Op::Get && q.path == open_) {
        r.status = Status::IoError;
      } else {
        open_.clear();
        r.status = q.op == Op::Get ? Status::NotFound
                   : q.op == Op::Delete ? Status::Ok
                                        : Status::IoError;
      }
    } else {
      apply(q, r, data_[slot]);
    }
    if (f && f->what == What::Drop) {
      resp = Msg{};  // the card did it; the reply never came
      return false;
    }
    return true;
  }

  void apply(const FileTransfer& q, FileTransfer& r, uint8_t* out) {
    std::string p = q.path;
    open_.clear();
    switch (q.op) {
      case Op::Get: {
        auto it = files.find(p);
        if (it == files.end()) {
          r.status = dirs.count(p) ? Status::NotAFile : Status::NotFound;
          return;
        }
        open_ = p;
        const Bytes& b = it->second;
        r.file_size = b.size();
        if (q.offset > b.size()) {
          r.status = Status::OffsetConflict;
          return;
        }
        size_t want = q.length == 0 || q.length > kMaxChunk ? kMaxChunk : q.length;
        size_t got = std::min(want, (size_t)(b.size() - q.offset));
        memcpy(out, b.data() + q.offset, got);
        r.data = got ? out : nullptr;
        r.data_len = (uint16_t)got;
        r.status = Status::Ok;
        return;
      }
      case Op::Post:
      case Op::Put: {
        if (q.op == Op::Put) putSizes.push_back(q.data_len);
        if (q.op == Op::Post) files.erase(p);
        if (dirs.count(p) || !dirs.count(parentOf(p))) {
          r.status = Status::IoError;  // the bridge's open for writing fails
          return;
        }
        Bytes& b = files[p];
        if (b.size() != q.offset) {
          r.status = Status::OffsetConflict;
          r.file_size = b.size();
          return;
        }
        b.insert(b.end(), q.data, q.data + q.data_len);
        r.file_size = b.size();
        r.status = Status::Ok;
        return;
      }
      case Op::Mkdir: {
        if (files.count(p)) {
          r.status = Status::NotAFile;
          return;
        }
        for (size_t i = p.find('/', 1);; i = p.find('/', i + 1)) {
          dirs.insert(i == std::string::npos ? p : p.substr(0, i));
          if (i == std::string::npos) break;
        }
        r.status = Status::Ok;
        return;
      }
      case Op::Delete:
        files.erase(p);
        r.status = Status::Ok;
        return;
    }
    r.status = Status::IoError;
  }
};

// The core and the fake are large: keep them off the stack.
static FakeCard* g_card;
static SdCore* g_core;
static uint8_t g_window[indicator::kSdReadAheadBytes];
static uint8_t g_buf[256 * 1024];

static FakeCard& fresh() {
  delete g_core;
  delete g_card;
  g_card = new FakeCard();
  g_core = new SdCore(*g_card, g_window);
  return *g_card;
}

static void writeFile(SdCore& sd, const char* path, const Bytes& b, size_t piece = 1024) {
  assert(sd.writeBegin(path));
  for (size_t off = 0; off < b.size(); off += piece)
    assert(sd.writeAppend(b.data() + off, std::min(piece, b.size() - off)));
  assert(sd.writeEnd());
}

// A write that fails somewhere: writeAppend and writeEnd report it.
static void failWrite(SdCore& sd, const char* path, const Bytes& b) {
  assert(sd.writeBegin(path));
  bool ok = true;
  for (size_t off = 0; off < b.size(); off += 1024) ok = sd.writeAppend(b.data() + off, 1024) && ok;
  assert(!ok);
  assert(!sd.writeEnd());
}

static void stat_and_chunked_read_to_eof() {
  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  Bytes tile = pattern(9123, 3);
  card.addFile("/maps/osm/7/66/43.png", tile);
  uint64_t size = 99;
  assert(sd.stat("/maps/osm/7/66/43.png", size) == 1 && size == 9123);
  assert(sd.readAt("/maps/osm/7/66/43.png", 0, g_buf, sizeof g_buf) == 9123);
  assert(memcmp(g_buf, tile.data(), tile.size()) == 0);
  assert(card.n(Op::Get) <= 3);
  assert(sd.stat("/maps/osm", size) == 2);
  assert(sd.stat("/nope.png", size) == 0 && size == 0);
  const int gets = card.n(Op::Get);
  assert(sd.stat("/", size) == 2);
  assert(card.n(Op::Get) == gets);  // the root needs no request

  // A short reply is the end of the file; reads at the end need no request.
  Bytes b = pattern(5000, 9);
  FakeCard& c2 = fresh();
  c2.addFile("/f", b);
  assert(g_core->readAt("/f", 4000, g_buf, 4096) == 1000);
  assert(memcmp(g_buf, b.data() + 4000, 1000) == 0);
  assert(g_core->readAt("/f", 0, g_buf, sizeof g_buf) == 5000);
  assert(memcmp(g_buf, b.data(), b.size()) == 0);
  const int gets2 = c2.n(Op::Get);
  assert(g_core->readAt("/f", 5000, g_buf, 10) == 0);
  assert(c2.n(Op::Get) == gets2);

  // An empty file (a reboot between a write's POST and its first PUT leaves one).
  FakeCard& c0 = fresh();
  c0.addFile("/tiles/e.jpg", Bytes{});
  assert(g_core->stat("/tiles/e.jpg", size) == 1 && size == 0);
  assert(g_core->readAt("/tiles/e.jpg", 0, g_buf, 10) == 0);

  // Past the end on a fresh core: the bridge answers OFFSET_CONFLICT, which is EOF.
  fresh().addFile("/f", b);
  assert(g_core->readAt("/f", 6000, g_buf, 10) == 0);

  // A file of whole chunks.
  FakeCard& c4 = fresh();
  Bytes even = pattern(2 * kMaxChunk, 2);
  c4.addFile("/even", even);
  assert(g_core->readAt("/even", 0, g_buf, sizeof g_buf) == (int)even.size());
  assert(memcmp(g_buf, even.data(), even.size()) == 0);
  assert(c4.n(Op::Get) <= 2);
}

// Once the size is known, the window is refilled with GETs three in flight,
// assembled in order. A lost reply, a nack for a garbled frame or a BUSY at the
// head of a batch costs only a retry of that chunk.
static void readahead_pipelines_in_order() {
  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  Bytes big = pattern(64 * 1024, 5);
  card.addFile("/big", big);
  for (size_t off = 0; off < big.size(); off += 1024) {
    assert(sd.readAt("/big", off, g_buf, 1024) == 1024);
    assert(memcmp(g_buf, big.data() + off, 1024) == 0);
  }
  assert(card.n(Op::Get) <= 16 + 1);
  assert(card.batches >= 2 && card.maxGetWindow == indicator::kSdGetWindow);
  const int gets = card.n(Op::Get);
  assert(sd.readAt("/big", big.size(), g_buf, 1024) == 0);  // EOF, known without asking
  assert(card.n(Op::Get) == gets);

  const FakeCard::Fault faults[] = {
      {Op::Get, 2, FakeCard::What::Drop, Status::Unspecified},         // head of the batch
      {Op::Get, 2, FakeCard::What::GarbledNack, Status::Unspecified},  // head of the batch
      {Op::Get, 3, FakeCard::What::GarbledNack, Status::Unspecified},  // inside it
      {Op::Get, 2, FakeCard::What::Reply, Status::Busy},
  };
  for (const auto& f : faults) {
    FakeCard& c = fresh();
    c.addFile("/big", big);
    assert(g_core->readAt("/big", 0, g_buf, 100) == 100);  // learns the size
    c.faults = {f};
    assert(g_core->readAt("/big", 100, g_buf, big.size()) == (int)big.size() - 100);
    assert(memcmp(g_buf, big.data() + 100, big.size() - 100) == 0);
    assert(c.slept <= indicator::kSdBusyWaitMs);
    assert(g_core->writesSupported());
  }
}

static void negative_cache_and_invalidation() {
  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  uint64_t size;
  // Missing files and directories are remembered.
  assert(sd.stat("/tiles/1/2/3.png", size) == 0);
  assert(sd.stat("/tiles/1/2/3.png", size) == 0);
  assert(sd.readAt("/tiles/1/2/3.png", 0, g_buf, 10) == -1);
  card.dirs.insert("/maps");
  assert(sd.stat("/maps", size) == 2);
  assert(sd.stat("/maps", size) == 2);
  assert(card.n(Op::Get) == 2);

  // probe asks for one byte and shares the cache; a file in the read-ahead
  // window needs no request.
  card.addFile("/maps/a.png", pattern(9000, 6));
  assert(sd.probe("/maps/a.png", size) == 1 && size == 9000 && card.lastGetLength == 1);
  assert(sd.probe("/tiles/1/2/3.png", size) == 0 && sd.probe("/maps", size) == 2);
  assert(sd.stat("/maps/a.png", size) == 1);
  const int gets = card.n(Op::Get);
  assert(sd.probe("/maps/a.png", size) == 1 && size == 9000);
  assert(card.n(Op::Get) == gets);

  // A ring of 256: the 257th miss pushes out the oldest.
  FakeCard& c2 = fresh();
  char p[32];
  for (int i = 0; i <= 256; ++i) {
    snprintf(p, sizeof p, "/miss/%d", i);
    assert(g_core->stat(p, size) == 0);
  }
  assert(g_core->stat("/miss/256", size) == 0 && g_core->stat("/miss/1", size) == 0);
  assert(c2.n(Op::Get) == 257);
  assert(g_core->stat("/miss/0", size) == 0);
  assert(c2.n(Op::Get) == 258);

  // Entries expire after kSdNegativeTtlMs.
  FakeCard& c3 = fresh();
  assert(g_core->stat("/x", size) == 0);
  c3.addFile("/x", pattern(7, 1));
  c3.clock += indicator::kSdNegativeTtlMs - 1;
  assert(g_core->stat("/x", size) == 0);
  c3.clock += 1;
  assert(g_core->stat("/x", size) == 1 && size == 7);

  // A write, a rewrite and a remove keep the caches true.
  FakeCard& c4 = fresh();
  c4.dirs.insert("/tiles");
  assert(g_core->stat("/tiles/n.png", size) == 0);
  writeFile(*g_core, "/tiles/n.png", pattern(300, 1));
  assert(g_core->stat("/tiles/n.png", size) == 1 && size == 300);
  assert(g_core->readAt("/tiles/n.png", 0, g_buf, 300) == 300);
  Bytes b2 = pattern(200, 77);
  writeFile(*g_core, "/tiles/n.png", b2);
  assert(g_core->readAt("/tiles/n.png", 0, g_buf, 300) == 200);
  assert(memcmp(g_buf, b2.data(), 200) == 0);
  assert(g_core->remove("/tiles/n.png"));
  assert(g_core->stat("/tiles/n.png", size) == 0);
  assert(g_core->remove("/tiles/n.png"));  // already gone is fine

  // mkdir forgets the misses of the new directory and its parents only.
  FakeCard& c5 = fresh();
  for (const char* d : {"/tiles", "/tiles/5", "/tiles/5/16", "/til"}) assert(g_core->stat(d, size) == 0);
  assert(g_core->mkdir("/tiles/5/16"));
  for (const char* d : {"/tiles", "/tiles/5", "/tiles/5/16"}) assert(g_core->stat(d, size) == 2);
  assert(c5.n(Op::Get) == 6);  // the two parents asked again, the new one known
  assert(g_core->stat("/til", size) == 0);
  assert(c5.n(Op::Get) == 6);

  // invalidate drops the negative, mkdir and read-ahead caches.
  FakeCard& c6 = fresh();
  Bytes r = pattern(3000, 6);
  c6.addFile("/r", r);
  assert(g_core->stat("/m", size) == 0);
  assert(g_core->mkdir("/d"));
  assert(g_core->readAt("/r", 0, g_buf, 100) == 100);
  g_core->invalidate();
  c6.addFile("/m", pattern(5, 5));
  const int gets6 = c6.n(Op::Get);
  assert(g_core->stat("/m", size) == 1 && size == 5);
  assert(g_core->mkdir("/d") && c6.n(Op::Mkdir) == 2);
  assert(g_core->readAt("/r", 200, g_buf, 100) == 100);
  assert(memcmp(g_buf, r.data() + 200, 100) == 0);
  assert(c6.n(Op::Get) == gets6 + 2);

  // So does another card seen by pollInfo; the same card keeps them.
  FakeCard& c7 = fresh();
  SdInfo info;
  assert(g_core->pollInfo(info) && info.present);
  assert(g_core->stat("/m", size) == 0);
  assert(g_core->pollInfo(info));
  const int gets7 = c7.n(Op::Get);
  assert(g_core->stat("/m", size) == 0);
  assert(c7.n(Op::Get) == gets7);
  c7.info.card_size = 16ull << 30;
  c7.addFile("/m", pattern(5, 5));
  assert(g_core->pollInfo(info));
  assert(g_core->stat("/m", size) == 1);
}

static void mkdir_cached() {
  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  assert(sd.mkdir("/tiles/5"));
  assert(sd.mkdir("/tiles/5"));
  assert(sd.mkdir("/"));
  assert(card.n(Op::Mkdir) == 1);
  assert(card.dirs.count("/tiles") && card.dirs.count("/tiles/5"));
  uint64_t size;
  assert(sd.stat("/tiles/5", size) == 2);
  card.addFile("/tiles/x", pattern(1, 1));  // a file in the way
  assert(!sd.mkdir("/tiles/x"));
  assert(sd.writesSupported());
}

// BUSY is waited out (kSdBusyWaitMs apart, up to the retry budget) and never
// cached as missing; right after NO_CARD it means the slot is still empty.
static void busy_is_retried_not_cached() {
  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  card.addFile("/a", pattern(100, 4));
  card.busyNext = 3;
  uint64_t size = 0;
  assert(sd.stat("/a", size) == 1 && size == 100);
  assert(card.n(Op::Get) == 4 && card.slept == 3 * indicator::kSdBusyWaitMs);

  card.busyForever = true;
  card.slept = 0;
  sd.invalidate();
  assert(sd.stat("/a", size) == -1);
  assert(card.n(Op::Get) == 4 + indicator::kSdBusyRetries + 1);
  assert(card.slept == indicator::kSdBusyRetries * indicator::kSdBusyWaitMs);
  card.busyForever = false;
  assert(sd.stat("/a", size) == 1);

  sd.invalidate();
  card.busyNext = 40;
  sd.setBusyRetries(2);
  const int gets = card.n(Op::Get);
  assert(sd.stat("/a", size) == -1);
  assert(card.n(Op::Get) == gets + 3);
  sd.setBusyRetries(-5);  // clamps to 0
  assert(sd.stat("/a", size) == -1);
  assert(card.n(Op::Get) == gets + 4);
  sd.setBusyRetries(indicator::kSdBusyRetries);

  FakeCard& c2 = fresh();
  c2.addFile("/a", pattern(10, 1));
  c2.noCard = true;
  assert(g_core->stat("/a", size) == -1);
  c2.noCard = false;
  c2.busyForever = true;  // the bridge keeps retrying the mount of an empty slot
  assert(g_core->stat("/a", size) == -1);
  assert(c2.slept == 0);
  c2.busyForever = false;
  assert(g_core->stat("/a", size) == 1);  // a real answer ends the no-card spell
}

// A lost reply is asked once more; two in a row are an error, not a missing file.
static void lost_reply_retried_once() {
  FakeCard& card = fresh();
  card.addFile("/a", pattern(10, 1));
  card.faults = {{Op::Get, 1, FakeCard::What::Drop, Status::Unspecified}};
  uint64_t size;
  assert(g_core->stat("/a", size) == 1 && size == 10);
  card.faults = {{Op::Get, 3, FakeCard::What::Drop, Status::Unspecified},
                 {Op::Get, 4, FakeCard::What::Drop, Status::Unspecified}};
  g_core->invalidate();
  assert(g_core->stat("/a", size) == -1);
  assert(card.n(Op::Get) == 4);
  assert(g_core->stat("/a", size) == 1);
}

// Appends are buffered into 4 KB PUTs, one request at a time, flushed by
// writeEnd. Requests that write wait kSdWriteReplyTimeoutMs for their reply
// (the bridge can take seconds to allocate a cluster); GETs keep the link's.
static void write_buffers_into_4k_puts() {
  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  assert(sd.mkdir("/tiles"));
  Bytes b = pattern(10 * 1024, 11);
  writeFile(sd, "/tiles/t.png", b);
  assert(card.n(Op::Post) == 1 && card.n(Op::Put) == 3);
  assert((card.putSizes == std::vector<uint32_t>{4096, 4096, 2048}));
  assert(card.files["/tiles/t.png"] == b);
  assert(!card.writeInBatch);
  uint64_t size;
  assert(sd.stat("/tiles/t.png", size) == 1 && size == b.size());

  Bytes c = pattern(9000, 12);
  writeFile(sd, "/tiles/u.png", c, c.size());  // one large append
  assert(card.files["/tiles/u.png"] == c);
  assert(sd.writeBegin("/tiles/e"));
  assert(sd.writeEnd());
  assert(card.files.count("/tiles/e") && card.files["/tiles/e"].empty());
  assert(sd.remove("/tiles/e"));

  assert(card.timeout_ms[Op::Get] == kReplyTimeoutMs);
  for (Op op : {Op::Mkdir, Op::Post, Op::Put, Op::Delete})
    assert(card.timeout_ms[op] == indicator::kSdWriteReplyTimeoutMs);
}

// A PUT whose reply was lost is sent again and meets OFFSET_CONFLICT: a file
// longer by exactly that chunk means the first try landed. Any other conflict,
// or a reply lost twice, fails the write and removes the partial file.
static void put_offset_conflict_resync() {
  Bytes b = pattern(10 * 1024, 13);
  FakeCard& card = fresh();
  card.dirs.insert("/tiles");
  card.faults = {{Op::Put, 2, FakeCard::What::Drop, Status::Unspecified}};
  writeFile(*g_core, "/tiles/l.png", b);
  assert(card.n(Op::Put) == 4);
  assert(card.files["/tiles/l.png"] == b);

  FakeCard& c2 = fresh();
  c2.dirs.insert("/tiles");
  c2.faults = {{Op::Put, 1, FakeCard::What::Reply, Status::OffsetConflict}};
  failWrite(*g_core, "/tiles/g.png", b);
  assert(!c2.files.count("/tiles/g.png"));

  FakeCard& c3 = fresh();
  c3.dirs.insert("/tiles");
  c3.faults = {{Op::Put, 2, FakeCard::What::Drop, Status::Unspecified},
               {Op::Put, 3, FakeCard::What::Drop, Status::Unspecified}};
  failWrite(*g_core, "/tiles/p.png", b);
  assert(c3.n(Op::Delete) == 1 && !c3.files.count("/tiles/p.png"));
}

// Half a file is never served: a failed write deletes its partial file, the
// path of the open write reads as missing, and a cleanup delete that did not
// get through is kept pending (the path still reads as missing) and sent again
// once a request has succeeded.
static void failed_write_leaves_no_partial() {
  Bytes b = pattern(10 * 1024, 14);
  uint64_t size;
  FakeCard& card = fresh();
  card.dirs.insert("/tiles");
  card.faults = {{Op::Put, 2, FakeCard::What::Reply, Status::IoError}};
  failWrite(*g_core, "/tiles/f.png", b);
  assert(!card.files.count("/tiles/f.png"));
  assert(card.n(Op::Delete) == 1 && card.n(Op::Put) == 2);  // nothing more after the failure

  // A write ended without committing (a caller lost bytes above the core): nothing
  // buffered is sent and the partial file is deleted.
  FakeCard& c1 = fresh();
  c1.dirs.insert("/tiles");
  assert(g_core->writeBegin("/tiles/g.png"));
  assert(g_core->writeAppend(b.data(), 5000));  // one PUT sent, the rest buffered
  const int puts = c1.n(Op::Put);
  assert(!g_core->writeEnd(false));
  assert(c1.n(Op::Put) == puts);
  assert(!c1.files.count("/tiles/g.png"));

  // A write left open is abandoned, and its partial file removed, by the next writeBegin.
  FakeCard& c2 = fresh();
  c2.dirs.insert("/tiles");
  assert(g_core->writeBegin("/tiles/h.png"));
  assert(g_core->writeAppend(b.data(), 5000));
  assert(g_core->writeBegin("/tiles/i.png"));
  assert(!c2.files.count("/tiles/h.png"));
  assert(g_core->writeEnd() && c2.files.count("/tiles/i.png"));

  // The open write's path, with an older copy on the card.
  FakeCard& c3 = fresh();
  c3.addFile("/tiles/0.jpg", pattern(100, 3));
  c3.addFile("/tiles/1.jpg", pattern(200, 5));
  assert(g_core->writeBegin("/tiles/0.jpg"));
  assert(g_core->writeAppend(b.data(), 5000));  // one PUT sent, the rest buffered
  const int gets = c3.n(Op::Get);
  assert(g_core->stat("/tiles/0.jpg", size) == 0);
  assert(g_core->probe("/tiles/0.jpg", size) == 0);
  assert(g_core->readAt("/tiles/0.jpg", 0, g_buf, 10) == -1);
  assert(c3.n(Op::Get) == gets);
  assert(g_core->stat("/tiles/1.jpg", size) == 1 && size == 200);
  assert(g_core->writeEnd());
  assert(g_core->readAt("/tiles/0.jpg", 0, g_buf, 5000) == 5000);
  assert(memcmp(g_buf, b.data(), 5000) == 0);

  // The cleanup delete answered NO_CARD.
  FakeCard& c4 = fresh();
  c4.addFile("/tiles/other.png", pattern(10, 1));
  c4.faults = {{Op::Put, 2, FakeCard::What::Reply, Status::IoError},
               {Op::Delete, 1, FakeCard::What::Reply, Status::NoCard}};
  failWrite(*g_core, "/tiles/f.png", b);
  assert(c4.files.count("/tiles/f.png"));
  const int gets4 = c4.n(Op::Get);
  assert(g_core->stat("/tiles/f.png", size) == 0);
  assert(g_core->readAt("/tiles/f.png", 0, g_buf, 10) == -1);
  assert(c4.n(Op::Get) == gets4);
  assert(g_core->stat("/tiles/other.png", size) == 1);  // the card answers again
  assert(c4.n(Op::Delete) == 1);
  assert(g_core->stat("/tiles/f.png", size) == 0);  // the delete goes ahead of this
  assert(c4.n(Op::Delete) == 2 && !c4.files.count("/tiles/f.png"));
  assert(g_core->stat("/tiles/other.png", size) == 1);
  assert(c4.n(Op::Delete) == 2);  // done once

  // A rewrite of the pending path truncates it: no delete needed.
  FakeCard& c5 = fresh();
  c5.dirs.insert("/tiles");
  c5.faults = {{Op::Put, 1, FakeCard::What::Reply, Status::IoError},
               {Op::Delete, 1, FakeCard::What::Drop, Status::Unspecified},
               {Op::Delete, 2, FakeCard::What::Drop, Status::Unspecified}};
  failWrite(*g_core, "/tiles/v.png", b);
  writeFile(*g_core, "/tiles/v.png", Bytes(b.begin(), b.begin() + 100));
  assert(c5.n(Op::Delete) == 2);  // only the two lost ones
  assert(g_core->stat("/tiles/v.png", size) == 1 && size == 100);
}

// The bridge has no card-detect line. A pulled card answers NOT_FOUND (IO_ERROR
// for its open file) until the bridge checks it, then BUSY while it remounts,
// or NO_CARD. Reads fail without a busy wait meanwhile, and what read as
// missing comes back once the card does.
static void card_pull_and_reseat() {
  uint64_t size;
  Bytes a = pattern(100, 1), big = pattern(64 * 1024, 2);
  {
    FakeCard& card = fresh();
    card.addFile("/maps/big.png", big);
    assert(g_core->readAt("/maps/big.png", 0, g_buf, 1024) == 1024);
    card.noCard = true;
    const int gets = card.n(Op::Get);
    assert(g_core->readAt("/maps/big.png", 40960, g_buf, 1024) == -1);
    assert(card.slept == 0);
    assert(card.n(Op::Get) - gets <= indicator::kSdReadAheadChunks);  // one batch, no retries
    assert(g_core->stat("/maps/big.png", size) == -1);
    card.noCard = false;
    assert(g_core->readAt("/maps/big.png", 40960, g_buf, 1024) == 1024);
    assert(memcmp(g_buf, big.data() + 40960, 1024) == 0);
  }
  {  // Quick reseat: the start of the BUSY spell drops the misses of the pulled card.
    FakeCard& card = fresh();
    card.addFile("/tiles/a.png", a);
    card.addFile("/tiles/b.png", pattern(50, 3));
    card.addFile("/tiles/open.png", big);
    assert(g_core->readAt("/tiles/open.png", 0, g_buf, 100) == 100);
    card.pulled = true;
    assert(g_core->stat("/tiles/a.png", size) == 0);
    assert(g_core->readAt("/tiles/open.png", 10000, g_buf, 100) == -1);
    card.pulled = false;
    card.busyNext = 20;
    assert(g_core->stat("/tiles/b.png", size) == 1 && size == 50);
    assert(g_core->stat("/tiles/a.png", size) == 1 && size == 100);
    assert(g_core->readAt("/tiles/open.png", 10000, g_buf, 100) == 100);
    assert(memcmp(g_buf, big.data() + 10000, 100) == 0);
  }
  {  // The same, seen as a BUSY behind good chunks of a batch.
    FakeCard& card = fresh();
    card.addFile("/tiles/a.png", a);
    card.addFile("/tiles/open.png", big);
    assert(g_core->readAt("/tiles/open.png", 0, g_buf, 100) == 100);
    card.faults = {{Op::Get, 2, FakeCard::What::Reply, Status::NotFound},
                   {Op::Get, 4, FakeCard::What::Reply, Status::Busy}};
    assert(g_core->stat("/tiles/a.png", size) == 0);
    assert(g_core->readAt("/tiles/open.png", 4096, g_buf, 100) == 100);
    assert(g_core->stat("/tiles/a.png", size) == 1);
  }
  {  // Pulled, then NO_CARD, then back.
    FakeCard& card = fresh();
    card.addFile("/tiles/a.png", a);
    card.pulled = true;
    assert(g_core->stat("/tiles/a.png", size) == 0);
    card.pulled = false;
    card.noCard = true;
    assert(g_core->stat("/tiles/x.png", size) == -1);
    card.noCard = false;
    assert(g_core->stat("/tiles/a.png", size) == 1);
  }
  {  // Pulled mid-write: a delete the pulled card answers OK proves nothing.
    FakeCard& card = fresh();
    card.addFile("/tiles/other.png", pattern(10, 1));
    Bytes b = pattern(10 * 1024, 16);
    assert(g_core->writeBegin("/tiles/w.png"));
    assert(g_core->writeAppend(b.data(), 4096));
    card.pulled = true;
    assert(!g_core->writeAppend(b.data() + 4096, b.size() - 4096));
    assert(!g_core->writeEnd());
    assert(card.files["/tiles/w.png"].size() == 4096);
    assert(g_core->remove("/tiles/old.png"));
    assert(g_core->stat("/tiles/w.png", size) == 0);
    card.pulled = false;
    card.busyNext = 5;
    assert(g_core->stat("/tiles/other.png", size) == 1);
    assert(g_core->stat("/tiles/w.png", size) == 0);
    assert(!card.files.count("/tiles/w.png"));
    writeFile(*g_core, "/tiles/w.png", b);
    assert(card.files["/tiles/w.png"] == b);
  }
}

// A bridge without MKDIR makes the card read-only: a nack matching a write
// request, or the stock firmware's IO_ERROR "unknown operation". A nack for a
// garbled frame (id 0) or any other IO_ERROR is a failed request, not a
// missing feature. The cleanup delete of an open write still goes out.
static void stock_bridge_is_read_only() {
  uint64_t size;
  for (int stock = 0; stock < 2; ++stock) {
    FakeCard& card = fresh();
    SdCore& sd = *g_core;
    (stock ? card.stockFirmware : card.nackMkdir) = true;
    card.addFile("/maps/osm/1/0/0.png", pattern(500, 2));
    assert(!sd.mkdir("/tiles"));
    assert(!sd.writesSupported());
    assert(!sd.mkdir("/tiles/1"));
    assert(!sd.writeBegin("/tiles/a.png"));
    uint8_t x = 1;
    assert(!sd.writeAppend(&x, 1) && !sd.writeEnd());
    assert(!sd.remove("/maps/osm/1/0/0.png"));
    assert(card.n(Op::Mkdir) == 1 && card.n(Op::Post) == 0 && card.n(Op::Put) == 0 &&
           card.n(Op::Delete) == 0);
    assert(sd.stat("/maps/osm/1/0/0.png", size) == 1 && size == 500);
    assert(sd.readAt("/maps/osm/1/0/0.png", 0, g_buf, 500) == 500);
    sd.invalidate();  // a cache reset is not a firmware change
    assert(!sd.writesSupported());
  }

  FakeCard& card = fresh();
  SdCore& sd = *g_core;
  card.faults = {{Op::Mkdir, 1, FakeCard::What::Reply, Status::IoError},
                 {Op::Mkdir, 2, FakeCard::What::GarbledNack, Status::Unspecified},
                 {Op::Mkdir, 4, FakeCard::What::GarbledNack, Status::Unspecified},
                 {Op::Mkdir, 5, FakeCard::What::GarbledNack, Status::Unspecified}};
  assert(!sd.mkdir("/tiles"));    // IO_ERROR
  assert(sd.mkdir("/tiles"));     // garbled nack, then OK on the retry
  assert(!sd.mkdir("/tiles/2"));  // garbled twice
  assert(sd.writesSupported());
  assert(sd.mkdir("/tiles/2"));
  card.faults = {{Op::Post, 1, FakeCard::What::Nack, Status::Unspecified}};
  assert(!sd.writeBegin("/tiles/2/a"));
  assert(!sd.writesSupported());

  FakeCard& c2 = fresh();
  c2.dirs.insert("/tiles");
  c2.nackMkdir = true;
  c2.faults = {{Op::Put, 2, FakeCard::What::Reply, Status::IoError}};
  Bytes b = pattern(10 * 1024, 19);
  assert(g_core->writeBegin("/tiles/z.png"));
  assert(g_core->writeAppend(b.data(), 5000));
  assert(!g_core->mkdir("/tiles/q"));
  assert(!g_core->writeAppend(b.data() + 5000, 4000));
  assert(!g_core->writeEnd());
  assert(c2.n(Op::Delete) == 1 && !c2.files.count("/tiles/z.png"));
}

int main() {
  stat_and_chunked_read_to_eof();
  readahead_pipelines_in_order();
  negative_cache_and_invalidation();
  mkdir_cached();
  busy_is_retried_not_cached();
  lost_reply_retried_once();
  write_buffers_into_4k_puts();
  put_offset_conflict_resync();
  failed_write_leaves_no_partial();
  card_pull_and_reseat();
  stock_bridge_is_read_only();
  delete g_core;
  delete g_card;
  printf("indicator sd core: all tests passed\n");
  return 0;
}
