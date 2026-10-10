// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// File access to the microSD card in the SenseCAP Indicator's RP2040 slot,
// through the bridge's file_transfer requests (Rp2040Msg.h). Reads are ranged
// 4 KB GETs, pipelined and kept in a read-ahead window; writes are a POST that
// creates the file and 4 KB PUT appends, one at a time. Header-only and
// hardware-free: requests go through a Requester, which the host test replaces
// with an in-memory card.
//
// Not thread-safe: the caller serialises every call on one SdCore, and holds
// that lock for the whole call, since replies point into the link's buffers.
// A call can wait out a busy card for its BUSY budget (setBusyRetries) times
// kSdBusyWaitMs while it holds that lock, so a caller blocked on the lock
// waits behind another caller's busy wait as well. The UI thread must take the
// lock with a try-lock (and draw the tile as missing when it fails), or keep SD
// calls off the UI thread; a small setBusyRetries() alone does not bound its
// wait.
//
// A pulled card: the bridge has no card-detect line. Until its card check
// runs (after 1 s without requests) a GET answers NOT_FOUND, or IO_ERROR for
// the file it holds open; then the check and the next mount answer BUSY. A
// card put back during that window mounts without NO_CARD ever being seen.
// So the start of every BUSY spell drops the caches, and negative entries
// expire after kSdNegativeTtlMs in any case.
//
// Memory: no allocation. The object holds the negative cache
// (kSdNegativeEntries paths), the mkdir cache, a 4 KB write buffer and the
// request/reply slots of one batch, about 35 KB; the read-ahead window
// (kSdReadAheadBytes) is the caller's buffer. Place both in PSRAM on the device.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Rp2040Link.h"
#include "Rp2040Msg.h"

namespace indicator {

// The link as SdCore sees it, so the host test can stand in for the bridge.
// call and callMany have LinkCore's semantics: a reply's file.data is valid
// until the next request; a lost reply leaves kind None, a nack kind Nack.
struct Requester {
  virtual bool call(const rp2040::Msg& req, rp2040::Msg& resp, uint32_t timeout_ms) = 0;
  // Replies that came in order: resps[0..result). resps[result] is None or Nack.
  virtual int callMany(const rp2040::Msg* reqs, rp2040::Msg* resps, int n, int window) = 0;
  virtual void sleepMs(uint32_t ms) = 0;
  virtual uint32_t nowMs() = 0;
  virtual ~Requester() = default;
};

constexpr size_t kSdChunk = rp2040::kMaxChunk;
constexpr int kSdReadAheadChunks = 8;
constexpr size_t kSdReadAheadBytes = kSdReadAheadChunks * kSdChunk;
// GETs in flight at once. Writes go one at a time: the bridge's 8 KB receive
// FIFO holds fewer than two full PUT frames.
constexpr int kSdGetWindow = 3;
// A card that is still mounting answers BUSY for about 2 s. The default
// budget of a core; setBusyRetries changes it.
constexpr int kSdBusyRetries = 30;
constexpr uint32_t kSdBusyWaitMs = 100;
// How long a request that writes (POST, PUT, DELETE, MKDIR) waits for its
// reply. The bridge allocates a cluster inside such a request, and SdFat
// looks for a free one from the start of the FAT after every remount (the
// bridge re-checks the card after 1 s without requests and after any
// NOT_FOUND), so a new file's first PUT can take seconds on a card whose
// free space lies far into the FAT. GETs keep rp2040::kReplyTimeoutMs.
constexpr uint32_t kSdWriteReplyTimeoutMs = 5000;
constexpr int kSdNegativeEntries = 256;
// How long a negative entry is believed, should no BUSY spell drop it first.
constexpr uint32_t kSdNegativeTtlMs = 60000;
constexpr int kSdMkdirEntries = 32;
// Longest path the caches keep; a longer one is always asked for.
constexpr size_t kSdCachedPathMax = 79;

static_assert(kSdReadAheadChunks <= rp2040::kMaxBatch, "one batch fills the window");

class SdCore {
public:
  // readahead: kSdReadAheadBytes of the caller's, which must outlive the core.
  SdCore(Requester& r, uint8_t* readahead) : r_(r), ra_(readahead) {}
  SdCore(const SdCore&) = delete;
  SdCore& operator=(const SdCore&) = delete;

  // 1 a file (size set), 2 a directory, 0 missing, -1 error (no card, a busy
  // card that stays busy, lost replies). A file's first chunk lands in the
  // read-ahead window, so the read that usually follows costs nothing more.
  // The path of the open write reads as missing (readAt fails on it), so no
  // one reads, or deletes, half a file.
  int stat(const char* path, uint64_t& size) {
    size = 0;
    if (!validPath(path)) return -1;
    if (isRoot(path)) return 2;
    if (isWriting(path)) return 0;  // not there until its write ends
    servicePending();
    if (isPending(path)) return 0;  // a partial file on its way out
    switch (negKind(path)) {
      case kMissing: return 0;
      case kDir: return 2;
      default: break;
    }
    if (inMkdir(path)) return 2;
    rp2040::Status st;
    int got = getOne(path, 0, st);
    if (got >= 0) {
      size = ra_size_;
      return 1;
    }
    if (st == rp2040::Status::NotAFile) return 2;
    if (st == rp2040::Status::NotFound) return 0;
    return -1;
  }

  // stat without the prefetch: a GET of one byte, for a caller that only asks
  // whether the path is there. Same results; size is known for a file.
  int probe(const char* path, uint64_t& size) {
    size = 0;
    if (!validPath(path)) return -1;
    if (isRoot(path)) return 2;
    if (isWriting(path)) return 0;
    if (raSameFile(path) && ra_has_size_) {
      size = ra_size_;
      return 1;
    }
    servicePending();
    if (isPending(path)) return 0;
    switch (negKind(path)) {
      case kMissing: return 0;
      case kDir: return 2;
      default: break;
    }
    if (inMkdir(path)) return 2;
    rp2040::Msg& q = fileReq(rp2040::Op::Get, path);
    q.file.length = 1;
    if (call(q, resps_[0]) != kReply) return -1;
    const rp2040::FileTransfer& f = resps_[0].file;
    switch (f.status) {
      case rp2040::Status::Ok:
      case rp2040::Status::OffsetConflict:  // an empty file
        size = f.file_size;
        return 1;
      case rp2040::Status::NotFound:
      case rp2040::Status::NotAFile:
        negNote(path, f.status);
        return f.status == rp2040::Status::NotAFile ? 2 : 0;
      default:
        return -1;
    }
  }

  // Up to n bytes from offset: the count, 0 at or past the end, -1 on any
  // error, even after part of the range was read. Sequential reads of the
  // most recent path come from the read-ahead window, without a look at the
  // negative cache.
  int readAt(const char* path, uint64_t offset, uint8_t* buf, size_t n) {
    if (!validPath(path) || (n > 0 && !buf) || isWriting(path)) return -1;
    if (!raHolds(path, offset)) {
      servicePending();
      if (isPending(path) || negKind(path) != kNone) return -1;
    }
    if (n > (size_t)INT32_MAX) n = (size_t)INT32_MAX;
    size_t done = 0;
    while (done < n) {
      uint64_t pos = offset + done;
      if (!raHolds(path, pos)) {
        int got = fill(path, pos);
        if (got < 0) return -1;
        if (got == 0) break;  // the end of the file
      }
      size_t at = (size_t)(pos - ra_off_);
      size_t take = ra_len_ - at;
      if (take > n - done) take = n - done;
      memcpy(buf + done, ra_ + at, take);
      done += take;
    }
    return (int)done;
  }

  // Creates or truncates path. An open write is abandoned first.
  bool writeBegin(const char* path) {
    if (writing_) abortWrite();
    if (!writes_ok_ || !validPath(path)) return false;
    forget(path);
    rp2040::Msg& q = fileReq(rp2040::Op::Post, path);
    if (call(q, resps_[0]) != kReply || resps_[0].file.status != rp2040::Status::Ok) return false;
    // The POST truncated a partial file of the same name; and it proved the
    // card is there, so a pending delete of another one goes now.
    if (isPending(path)) pend_path_[0] = '\0';
    servicePending();
    strcpy(wpath_, path);
    writing_ = true;
    wfailed_ = false;
    woff_ = 0;
    wlen_ = 0;
    return true;
  }

  // Buffers data and sends each full 4 KB as a PUT. False once the write has
  // failed; writeEnd then removes the partial file.
  bool writeAppend(const uint8_t* data, size_t n) {
    if (!writing_ || wfailed_) return false;
    if (n > 0 && !data) {
      wfailed_ = true;
      return false;
    }
    while (n > 0) {
      size_t take = kSdChunk - wlen_;
      if (take > n) take = n;
      memcpy(wbuf_ + wlen_, data, take);
      wlen_ += take;
      data += take;
      n -= take;
      if (wlen_ == kSdChunk && !putChunk()) {
        wfailed_ = true;
        return false;
      }
    }
    return true;
  }

  // Sends what is buffered; with commit false, sends nothing and ends the write
  // as failed. On any failure of this write the result is false and the
  // partial file is deleted. Every PUT closes the file on the card, so
  // what was sent stays there; when the card is gone, or seems to be, the
  // delete is kept pending and sent again once a request has succeeded, and
  // until then the path reads as missing.
  bool writeEnd(bool commit = true) {
    if (!writing_) return false;
    bool ok = commit && !wfailed_ && (wlen_ == 0 || putChunk());
    writing_ = false;
    forget(wpath_);
    if (!ok) dropPartial(wpath_);
    return ok;
  }

  // One directory, its parents included. Remembered once made.
  bool mkdir(const char* path) {
    if (!writes_ok_ || !validPath(path)) return false;
    if (isRoot(path) || inMkdir(path)) return true;
    rp2040::Msg& q = fileReq(rp2040::Op::Mkdir, path);
    if (call(q, resps_[0]) != kReply || resps_[0].file.status != rp2040::Status::Ok) return false;
    negForgetPrefixes(path);
    mkdirAdd(path);
    return true;
  }

  // Deletes a file; one that is not there counts as deleted.
  bool remove(const char* path) {
    if (!writes_ok_ || !validPath(path)) return false;
    forget(path);
    mkdirForget(path);
    return deleteFile(path) == kDeleted;
  }

  // The bridge's card info. A card that appears, goes or changes (another
  // card_size), and a busy card, drop the caches.
  bool pollInfo(rp2040::SdInfo& out) {
    servicePending();
    rp2040::Msg& q = reqs_[0];
    q = rp2040::Msg{};
    q.kind = rp2040::Kind::GetSdInfo;
    if (!r_.call(q, resps_[0], rp2040::kReplyTimeoutMs)) return false;
    if (resps_[0].kind != rp2040::Kind::SdInfo) return false;
    out = resps_[0].sd;
    if (have_info_ && (out.present != last_present_ || out.card_size != last_card_size_))
      invalidate();
    have_info_ = true;
    last_present_ = out.present;
    last_card_size_ = out.card_size;
    if (out.busy)
      noteBusy(false);
    else if (out.present)
      busy_run_ = false;
    if (out.present)
      no_card_ = false;
    else if (!out.busy)
      setNoCard();
    return true;
  }

  // Drops the negative, mkdir and read-ahead caches. A NO_CARD reply, the
  // start of a BUSY spell and a card change seen by pollInfo do this too. A
  // pending delete stays.
  void invalidate() {
    dropCaches(false);
  }

  // False once the bridge nacked a write request or answered it with "unknown
  // operation": its firmware has no MKDIR (or no writes). Reads still work.
  bool writesSupported() const { return writes_ok_; }

  // BUSY replies one request waits out, kSdBusyWaitMs apart, before it fails.
  void setBusyRetries(int n) { busy_retries_ = n < 0 ? 0 : n; }

private:
  enum NegKind : uint8_t { kNone, kMissing, kDir };
  enum Outcome { kReply, kLost, kUnsupported, kBusy, kNoCard };
  enum DeleteResult { kDeleted, kDeleteUnsupported, kDeleteFailed };

  struct NegEntry {
    char path[kSdCachedPathMax + 1];
    uint8_t kind;
    uint32_t at_ms;
  };

  static bool isUnknownOp(const char* message) {
    static const char kUnknownOp[] = "unknown operation";
    return strncmp(message, kUnknownOp, sizeof kUnknownOp - 1) == 0;
  }

  static bool validPath(const char* p) { return p && p[0] && strlen(p) <= rp2040::kMaxPath; }
  static bool isRoot(const char* p) { return p[0] == '/' && p[1] == '\0'; }

  // --- requests ------------------------------------------------------------ //

  rp2040::Msg& buildReq(rp2040::Op op, const char* path, int slot = 0) {
    rp2040::Msg& q = reqs_[slot];
    q = rp2040::Msg{};
    q.kind = rp2040::Kind::File;
    q.file.op = op;
    strcpy(q.file.path, path);
    return q;
  }

  // A request in reqs_[0], after any pending delete that is due.
  rp2040::Msg& fileReq(rp2040::Op op, const char* path) {
    servicePending();
    return buildReq(op, path);
  }

  // A status other than BUSY ends a BUSY spell, and ends or starts a no-card
  // spell. Going into one drops the caches: whatever card comes next is not
  // known.
  void noteStatus(rp2040::Status s) {
    if (s == rp2040::Status::Busy) return;
    busy_run_ = false;
    if (s == rp2040::Status::NoCard)
      setNoCard();
    else
      no_card_ = false;
  }

  void setNoCard() {
    if (!no_card_) invalidate();
    no_card_ = true;
  }

  // The start of a BUSY spell: the bridge is checking or mounting a card (or
  // scanning it), so the NOT_FOUND answers before it may have come from a
  // card that was being pulled. keep_window keeps chunks that arrived ahead
  // of the BUSY reply in the same batch.
  void noteBusy(bool keep_window) {
    if (busy_run_) return;
    busy_run_ = true;
    dropCaches(keep_window);
  }

  // A request other than a delete succeeded: the card is really there, so a
  // pending delete can be believed.
  void noteOk() {
    if (pend_path_[0]) pend_ready_ = true;
  }

  // One request the careful way: a lost reply (or a nack for a frame the
  // bridge could not decode, id 0) is sent once more, BUSY is waited out,
  // except right after NO_CARD, where it means the slot is still empty. A nack
  // that matched our request means the bridge does not know the operation;
  // for a write that turns writes off, as does an IO_ERROR "unknown
  // operation" answer. kReply leaves the reply in resp.
  Outcome call(const rp2040::Msg& req, rp2040::Msg& resp) {
    int lost = 0;
    int busy = 0;
    for (;;) {
      const bool writes = req.kind == rp2040::Kind::File && req.file.op != rp2040::Op::Get;
      if (!r_.call(req, resp, writes ? kSdWriteReplyTimeoutMs : rp2040::kReplyTimeoutMs)) {
        if (resp.kind == rp2040::Kind::Nack && resp.id != 0) {
          if (req.kind == rp2040::Kind::File && req.file.op != rp2040::Op::Get) writes_ok_ = false;
          return kUnsupported;
        }
        if (++lost > 1) return kLost;
        continue;
      }
      if (resp.kind != rp2040::Kind::File) return kLost;
      rp2040::Status s = resp.file.status;
      // Stock bridge firmware answers an operation it does not know (MKDIR)
      // with IO_ERROR and this message rather than a nack.
      if (s == rp2040::Status::IoError && req.kind == rp2040::Kind::File &&
          req.file.op != rp2040::Op::Get && isUnknownOp(resp.file.message)) {
        writes_ok_ = false;
        return kUnsupported;
      }
      if (s == rp2040::Status::Busy) {
        if (no_card_) return kNoCard;
        noteBusy(false);
        if (++busy > busy_retries_) return kBusy;
        r_.sleepMs(kSdBusyWaitMs);
        continue;
      }
      noteStatus(s);
      if (s == rp2040::Status::Ok && req.file.op != rp2040::Op::Delete) noteOk();
      return kReply;
    }
  }

  // --- reading ------------------------------------------------------------- //

  bool raSameFile(const char* path) const { return ra_valid_ && strcmp(ra_path_, path) == 0; }

  bool raHolds(const char* path, uint64_t pos) const {
    return raSameFile(path) && pos >= ra_off_ && pos < ra_off_ + ra_len_;
  }

  void raReset() {
    ra_valid_ = false;
    ra_has_size_ = false;
    ra_path_[0] = '\0';
    ra_off_ = 0;
    ra_len_ = 0;
    ra_size_ = 0;
  }

  // Empties the window and points it at path from pos; the size of the same
  // file stays known.
  void raStart(const char* path, uint64_t pos) {
    if (!raSameFile(path)) {
      raReset();
      strcpy(ra_path_, path);
      ra_valid_ = true;
    }
    ra_off_ = pos;
    ra_len_ = 0;
  }

  // Takes one chunk reply (status OK) into the window. False at the end of the
  // file, which only a short reply tells.
  bool raTake(const rp2040::FileTransfer& f) {
    if (f.data_len > 0) memcpy(ra_ + ra_len_, f.data, f.data_len);
    uint64_t end = ra_off_ + ra_len_ + f.data_len;
    ra_len_ += f.data_len;
    ra_size_ = f.file_size;
    ra_has_size_ = true;
    if (f.data_len < kSdChunk) {
      ra_size_ = end;
      return false;
    }
    return true;
  }

  // One chunk at pos, the careful way, into a fresh window: its length (0 at
  // the end), or -1 with st set (Unspecified when there was no answer).
  int getOne(const char* path, uint64_t pos, rp2040::Status& st) {
    st = rp2040::Status::Unspecified;
    rp2040::Msg& q = fileReq(rp2040::Op::Get, path);
    q.file.offset = pos;
    q.file.length = kSdChunk;
    if (call(q, resps_[0]) != kReply) return -1;
    const rp2040::FileTransfer& f = resps_[0].file;
    st = f.status;
    if (st != rp2040::Status::Ok) {
      negNote(path, st);
      return -1;
    }
    raStart(path, pos);
    raTake(f);
    return (int)ra_len_;
  }

  // Remembers what a reply said is not a readable file.
  void negNote(const char* path, rp2040::Status st) {
    if (st == rp2040::Status::NotFound)
      negAdd(path, kMissing);
    else if (st == rp2040::Status::NotAFile)
      negAdd(path, kDir);
  }

  // Refills the window from pos: the bytes it now holds, 0 at the end of the
  // file, -1 on error. With the size known, the chunks up to the window's size
  // are asked for together; anything short of a clean answer to the first
  // falls back to asking for it alone.
  int fill(const char* path, uint64_t pos) {
    bool known = raSameFile(path) && ra_has_size_;
    if (known && pos >= ra_size_) return 0;
    int count = 1;
    if (known) {
      uint64_t chunks = (ra_size_ - pos + kSdChunk - 1) / kSdChunk;
      count = chunks < (uint64_t)kSdReadAheadChunks ? (int)chunks : kSdReadAheadChunks;
    }
    if (count > 1) {
      int r = fillMany(path, pos, count);
      if (r != kRetryAlone) return r;
    }
    rp2040::Status st;
    int got = getOne(path, pos, st);
    if (got >= 0) return got;
    return st == rp2040::Status::OffsetConflict ? 0 : -1;  // past the end
  }

  static constexpr int kRetryAlone = -2;

  int fillMany(const char* path, uint64_t pos, int count) {
    servicePending();
    for (int i = 0; i < count; ++i) {
      rp2040::Msg& q = buildReq(rp2040::Op::Get, path, i);
      q.file.offset = pos + (uint64_t)i * kSdChunk;
      q.file.length = kSdChunk;
    }
    int k = r_.callMany(reqs_, resps_, count, kSdGetWindow);
    if (k < 0) k = 0;
    if (k > count) k = count;
    // Copy out at once: the replies' data lives in the link until its next request.
    raStart(path, pos);
    int i = 0;
    for (; i < k; ++i) {
      if (resps_[i].kind != rp2040::Kind::File || resps_[i].file.status != rp2040::Status::Ok) break;
      if (!raTake(resps_[i].file)) {
        ++i;
        break;
      }
    }
    if (i > 0) {
      no_card_ = false;
      busy_run_ = false;
      noteOk();
      // A BUSY behind the chunks that came: the card is being checked.
      if (i < k && resps_[i].kind == rp2040::Kind::File &&
          resps_[i].file.status == rp2040::Status::Busy)
        noteBusy(true);
      return (int)ra_len_;
    }
    // Nothing usable. A clear answer about the file ends it here; BUSY, a lost
    // reply or a garbled frame is asked again on its own.
    if (k > 0) {
      const rp2040::FileTransfer& f = resps_[0].file;
      if (resps_[0].kind != rp2040::Kind::File) return -1;
      rp2040::Status st = f.status;
      if (st == rp2040::Status::Busy && no_card_) return -1;
      if (st == rp2040::Status::Busy) {
        noteBusy(false);
        return kRetryAlone;
      }
      noteStatus(st);
      negNote(path, st);
      return st == rp2040::Status::OffsetConflict ? 0 : -1;
    }
    if (resps_[0].kind == rp2040::Kind::Nack && resps_[0].id != 0) return -1;
    return kRetryAlone;
  }

  // --- writing ------------------------------------------------------------- //

  // The buffered chunk as a PUT at the write offset. After a lost reply the
  // retry finds the file already longer; when by exactly this chunk, the first
  // try went through.
  bool putChunk() {
    rp2040::Msg& q = fileReq(rp2040::Op::Put, wpath_);
    q.file.offset = woff_;
    q.file.data = wbuf_;
    q.file.data_len = (uint16_t)wlen_;
    if (call(q, resps_[0]) != kReply) return false;
    const rp2040::FileTransfer& f = resps_[0].file;
    bool ok = f.status == rp2040::Status::Ok ||
              (f.status == rp2040::Status::OffsetConflict && f.file_size == woff_ + wlen_);
    if (!ok) return false;
    woff_ += wlen_;
    wlen_ = 0;
    return true;
  }

  // Sent even with writes turned off: a write that was open when that
  // happened still leaves a partial file.
  DeleteResult deleteFile(const char* path) {
    rp2040::Msg& q = buildReq(rp2040::Op::Delete, path);
    Outcome o = call(q, resps_[0]);
    if (o == kUnsupported) return kDeleteUnsupported;
    if (o != kReply) return kDeleteFailed;
    rp2040::Status s = resps_[0].file.status;
    return s == rp2040::Status::Ok || s == rp2040::Status::NotFound ? kDeleted : kDeleteFailed;
  }

  // The partial file of a failed write: deleted now, and once more after a
  // request has succeeded, since a card being pulled answers a delete with
  // OK without doing it. The slot holds one path; a newer failure takes it.
  void dropPartial(const char* path) {
    servicePending();
    if (deleteFile(path) == kDeleteUnsupported) return;
    strcpy(pend_path_, path);
    pend_ready_ = false;
  }

  // Sends the pending delete once a request has succeeded since it was set.
  // Done on OK; any other answer waits for the next success.
  void servicePending() {
    if (!pend_path_[0] || !pend_ready_ || in_pending_) return;
    in_pending_ = true;
    DeleteResult d = deleteFile(pend_path_);
    in_pending_ = false;
    if (d == kDeleteFailed) {
      pend_ready_ = false;
      return;
    }
    pend_path_[0] = '\0';
    pend_ready_ = false;
  }

  bool isPending(const char* path) const { return pend_path_[0] && strcmp(pend_path_, path) == 0; }

  // The path of the open write: half a file, read as missing until it ends.
  bool isWriting(const char* path) const { return writing_ && strcmp(wpath_, path) == 0; }

  void abortWrite() {
    writing_ = false;
    forget(wpath_);
    dropPartial(wpath_);
  }

  // What the caches know about path, after a write to it.
  void forget(const char* path) {
    negForget(path);
    if (raSameFile(path)) raReset();
  }

  // --- caches -------------------------------------------------------------- //

  void dropCaches(bool keep_window) {
    for (auto& e : neg_) e.path[0] = '\0';
    neg_next_ = 0;
    for (auto& e : mkdir_) e[0] = '\0';
    mkdir_next_ = 0;
    if (!keep_window) raReset();
  }

  static bool cacheable(const char* path) { return strlen(path) <= kSdCachedPathMax; }

  uint8_t negKind(const char* path) {
    if (!cacheable(path)) return kNone;
    for (auto& e : neg_) {
      if (!e.path[0] || strcmp(e.path, path) != 0) continue;
      if (r_.nowMs() - e.at_ms >= kSdNegativeTtlMs) {
        e.path[0] = '\0';  // too old to believe
        return kNone;
      }
      return e.kind;
    }
    return kNone;
  }

  // A path in the negative cache is never in the read-ahead window, so a read
  // from the window needs no look at this cache.
  void negAdd(const char* path, uint8_t kind) {
    if (raSameFile(path)) raReset();
    if (!cacheable(path)) return;
    uint32_t now = r_.nowMs();
    for (auto& e : neg_) {
      if (e.path[0] && strcmp(e.path, path) == 0) {
        e.kind = kind;
        e.at_ms = now;
        return;
      }
    }
    NegEntry& e = neg_[neg_next_];
    strcpy(e.path, path);
    e.kind = kind;
    e.at_ms = now;
    neg_next_ = (neg_next_ + 1) % kSdNegativeEntries;
  }

  void negForget(const char* path) {
    if (!cacheable(path)) return;
    for (auto& e : neg_)
      if (e.path[0] && strcmp(e.path, path) == 0) e.path[0] = '\0';
  }

  // A new directory: it and every parent of it exist.
  void negForgetPrefixes(const char* dir) {
    for (auto& e : neg_) {
      if (!e.path[0]) continue;
      size_t n = strlen(e.path);
      if (strncmp(e.path, dir, n) == 0 && (dir[n] == '/' || dir[n] == '\0')) e.path[0] = '\0';
    }
  }

  bool inMkdir(const char* path) const {
    if (!cacheable(path)) return false;
    for (const auto& e : mkdir_)
      if (e[0] && strcmp(e, path) == 0) return true;
    return false;
  }

  void mkdirAdd(const char* path) {
    if (!cacheable(path)) return;
    strcpy(mkdir_[mkdir_next_], path);
    mkdir_next_ = (mkdir_next_ + 1) % kSdMkdirEntries;
  }

  void mkdirForget(const char* path) {
    if (!cacheable(path)) return;
    for (auto& e : mkdir_)
      if (e[0] && strcmp(e, path) == 0) e[0] = '\0';
  }

  Requester& r_;

  // Read-ahead window: ra_[0, ra_len_) is the file ra_path_ from ra_off_.
  uint8_t* ra_;
  bool ra_valid_ = false;
  bool ra_has_size_ = false;
  char ra_path_[rp2040::kMaxPath + 1] = {};
  uint64_t ra_off_ = 0;
  size_t ra_len_ = 0;
  uint64_t ra_size_ = 0;

  // Paths the card said are missing (kMissing) or directories (kDir), oldest
  // overwritten first.
  NegEntry neg_[kSdNegativeEntries] = {};
  int neg_next_ = 0;
  char mkdir_[kSdMkdirEntries][kSdCachedPathMax + 1] = {};
  int mkdir_next_ = 0;

  // The open write.
  bool writing_ = false;
  bool wfailed_ = false;
  char wpath_[rp2040::kMaxPath + 1] = {};
  uint64_t woff_ = 0;  // where the buffered chunk goes on the card
  size_t wlen_ = 0;
  uint8_t wbuf_[kSdChunk];

  // The partial file of a failed write that may still be on the card.
  char pend_path_[rp2040::kMaxPath + 1] = {};
  bool pend_ready_ = false;  // a request succeeded since: the delete can be believed
  bool in_pending_ = false;

  bool writes_ok_ = true;
  bool no_card_ = false;   // the last real answer was NO_CARD: BUSY means an empty slot
  bool busy_run_ = false;  // the last answer was BUSY
  int busy_retries_ = kSdBusyRetries;
  bool have_info_ = false;
  bool last_present_ = false;
  uint64_t last_card_size_ = 0;

  rp2040::Msg reqs_[kSdReadAheadChunks];
  rp2040::Msg resps_[kSdReadAheadChunks];
};

}  // namespace indicator
