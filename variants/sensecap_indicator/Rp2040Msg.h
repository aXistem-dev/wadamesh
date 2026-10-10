// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Messages of the SenseCAP Indicator's RP2040 bridge firmware, the subset the
// ESP32 speaks over UART: ping/pong, nack, file transfer, SD card info and beep. On
// the wire they are the bridge's InterdeviceMessage (interdevice.proto),
// coded with nanopb in Rp2040Codec.cpp; fields not modelled here are dropped on
// decode. encode and decode share one message, allocated on first use (in PSRAM
// on the device), so one caller at a time: the link calls them under its mutex.

#include <stddef.h>
#include <stdint.h>

namespace rp2040 {

// Largest payload of one frame on the link.
constexpr size_t kMaxPayload = 4666;
// FileTransfer.filepath, without the terminator, and FileTransfer.filedata.
constexpr size_t kMaxPath = 255;
constexpr size_t kMaxChunk = 4096;
// FileTransfer.message as kept here, without the terminator; a longer one is
// cut to this length on decode.
constexpr size_t kMaxMessage = 63;
// Version both sides carry in ping/pong.
constexpr uint32_t kVersion = 2;

enum class Op : uint32_t { Get = 0, Post = 1, Put = 2, Delete = 3, Mkdir = 4 };

enum class Status : uint32_t {
  Unspecified = 0,
  Ok = 1,
  Busy = 2,            // the bridge is mounting or scanning the card, try again
  NoCard = 3,
  NotFound = 4,
  OffsetConflict = 5,  // PUT: file_size holds the size the file really has
  IoError = 6,
  NotAFile = 7,
};

struct FileTransfer {
  Op op = Op::Get;
  char path[kMaxPath + 1] = {};
  // After decode, data points into the codec's message: valid until the next
  // encode or decode.
  const uint8_t* data = nullptr;
  uint16_t data_len = 0;
  Status status = Status::Unspecified;
  char message[kMaxMessage + 1] = {};   // the reply's human-readable detail, may be empty
  uint64_t offset = 0;
  uint32_t length = 0;      // GET request: bytes wanted, 0 = one full chunk
  uint64_t file_size = 0;
};

struct SdInfo {
  bool present = false;
  bool busy = false;
  bool unformatted = false;
  bool stats_valid = false;   // used/free are meaningful only when set
  uint8_t card_type = 0;      // SdCardInfo.CardType: 3 = SDHC, 4 = SDXC
  uint8_t fat_type = 0;       // SdCardInfo.FatType: 2 = FAT32, 3 = exFAT
  uint64_t card_size = 0;
  uint64_t used = 0;
  uint64_t free = 0;
};

// Which member of the oneof the message carries. Other is any member not
// modelled here (nmea, i2c, listing, ...); None is no member at all.
enum class Kind { None, Ping, Pong, Nack, File, GetSdInfo, SdInfo, Beep, Other };

struct Msg {
  Kind kind = Kind::None;
  uint32_t id = 0;        // a reply echoes the id of its request
  uint32_t version = 0;   // Ping, Pong
  FileTransfer file;      // File
  SdInfo sd;              // SdInfo
  uint16_t beep_ms = 0;   // Beep: buzzer on for this long, 0 = off
};

// Writes the request m (Ping, File, GetSdInfo or Beep) to out and returns its length,
// or 0 when it does not fit in cap, is of another kind, has a field out of
// range (path or message without terminator, data_len over kMaxChunk, data
// missing), or the shared message cannot be allocated.
size_t encode(const Msg& m, uint8_t* out, size_t cap);

// Parses n bytes of in into out. False on input nanopb rejects (truncated,
// malformed, or a string or bytes field over its interdevice.options limit) or
// when the shared message cannot be allocated. A message over kMaxMessage is
// cut, not rejected.
bool decode(const uint8_t* in, size_t n, Msg& out);

}  // namespace rp2040
