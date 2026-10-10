// SPDX-License-Identifier: GPL-3.0-or-later
//
// The RP2040 bridge messages of Rp2040Msg.h on nanopb: maps rp2040::Msg to and
// from meshtastic_InterdeviceMessage (proto/interdevice.pb.h), and nanopb does
// the wire format. One message (about 4.6 KB) serves both directions, so
// neither puts it on a stack; malloc places a block that size in PSRAM.

#include "Rp2040Msg.h"

#include <stdlib.h>
#include <string.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "proto/interdevice.pb.h"

namespace rp2040 {
namespace {

static_assert(sizeof(meshtastic_FileTransfer::filepath) == kMaxPath + 1, "filepath size");
static_assert(sizeof(meshtastic_FileTransfer::filedata.bytes) == kMaxChunk, "filedata size");
static_assert(sizeof(meshtastic_FileTransfer::message) > kMaxMessage, "message size");
static_assert(meshtastic_InterdeviceMessage_size <= kMaxPayload, "payload size");

meshtastic_InterdeviceMessage* s_pb = nullptr;

// The shared message, or nullptr when it cannot be allocated.
meshtastic_InterdeviceMessage* message() {
  if (!s_pb) s_pb = static_cast<meshtastic_InterdeviceMessage*>(malloc(sizeof *s_pb));
  return s_pb;
}

// nanopb keeps an enum field as the int32 it decoded, which can be a value the
// C enum does not list (a newer bridge). Its bits are read and written as a
// uint32_t, never through the enum type.
template <typename E>
uint32_t rawEnum(const E& e) {
  static_assert(sizeof(E) == sizeof(uint32_t), "nanopb enums are 32-bit");
  uint32_t v;
  memcpy(&v, &e, sizeof v);
  return v;
}

template <typename E>
void setEnum(E& e, uint32_t v) {
  static_assert(sizeof(E) == sizeof(uint32_t), "nanopb enums are 32-bit");
  memcpy(&e, &v, sizeof v);
}

// Copies the terminated string src into dst[cap + 1], cut to cap bytes.
void copyCut(char* dst, const char* src, size_t cap) {
  const size_t n = strnlen(src, cap);
  memcpy(dst, src, n);
  dst[n] = 0;
}

void fromFile(const meshtastic_FileTransfer& p, FileTransfer& f) {
  f.op = (Op)rawEnum(p.operation);
  copyCut(f.path, p.filepath, kMaxPath);   // nanopb rejects a longer one
  f.data_len = (uint16_t)p.filedata.size;
  f.data = f.data_len ? p.filedata.bytes : nullptr;
  f.status = (Status)rawEnum(p.status);
  copyCut(f.message, p.message, kMaxMessage);
  f.offset = p.offset;
  f.length = p.length;
  f.file_size = p.file_size;
}

void fromSd(const meshtastic_SdCardInfo& p, SdInfo& s) {
  s.present = p.present;
  s.busy = p.busy;
  s.unformatted = p.unformatted;
  s.stats_valid = p.stats_valid;
  s.card_type = (uint8_t)rawEnum(p.card_type);
  s.fat_type = (uint8_t)rawEnum(p.fat_type);
  s.card_size = p.card_size;
  s.used = p.used_bytes;
  s.free = p.free_bytes;
}

bool toFile(const FileTransfer& f, meshtastic_FileTransfer& p) {
  const size_t path_len = strnlen(f.path, sizeof f.path);
  const size_t message_len = strnlen(f.message, sizeof f.message);
  if (path_len > kMaxPath || message_len > kMaxMessage || f.data_len > kMaxChunk ||
      (f.data_len && !f.data))
    return false;
  memset(&p, 0, sizeof p);
  setEnum(p.operation, (uint32_t)f.op);
  memcpy(p.filepath, f.path, path_len);
  p.filedata.size = f.data_len;
  if (f.data_len) memcpy(p.filedata.bytes, f.data, f.data_len);
  setEnum(p.status, (uint32_t)f.status);
  memcpy(p.message, f.message, message_len);
  p.offset = f.offset;
  p.length = f.length;
  p.file_size = f.file_size;
  return true;
}

}  // namespace

size_t encode(const Msg& m, uint8_t* out, size_t cap) {
  meshtastic_InterdeviceMessage* pb = message();
  if (!pb) return 0;
  switch (m.kind) {
    case Kind::Ping:
      pb->which_data = meshtastic_InterdeviceMessage_ping_tag;
      setEnum(pb->data.ping, m.version);
      break;
    case Kind::GetSdInfo:
      pb->which_data = meshtastic_InterdeviceMessage_get_sd_info_tag;
      pb->data.get_sd_info = true;
      break;
    case Kind::File:
      pb->which_data = meshtastic_InterdeviceMessage_file_transfer_tag;
      if (!toFile(m.file, pb->data.file_transfer)) return 0;
      break;
    case Kind::Beep:
      pb->which_data = meshtastic_InterdeviceMessage_beep_tag;
      pb->data.beep = m.beep_ms;
      break;
    default:
      return 0;
  }
  pb->id = m.id;
  // proto3: zero scalars and empty strings are left out; the active oneof
  // member is always written, so an empty sub-message goes out as length 0.
  pb_ostream_t stream = pb_ostream_from_buffer(out, cap);
  if (!pb_encode(&stream, meshtastic_InterdeviceMessage_fields, pb)) return 0;
  return stream.bytes_written;
}

bool decode(const uint8_t* in, size_t n, Msg& out) {
  out = Msg{};
  meshtastic_InterdeviceMessage* pb = message();
  if (!pb) return false;
  pb_istream_t stream = pb_istream_from_buffer(in, n);
  if (!pb_decode(&stream, meshtastic_InterdeviceMessage_fields, pb)) return false;
  out.id = pb->id;
  switch (pb->which_data) {
    case 0:
      out.kind = Kind::None;
      break;
    case meshtastic_InterdeviceMessage_ping_tag:
      out.kind = Kind::Ping;
      out.version = rawEnum(pb->data.ping);
      break;
    case meshtastic_InterdeviceMessage_pong_tag:
      out.kind = Kind::Pong;
      out.version = rawEnum(pb->data.pong);
      break;
    case meshtastic_InterdeviceMessage_nack_tag:
      out.kind = Kind::Nack;
      break;
    case meshtastic_InterdeviceMessage_get_sd_info_tag:
      out.kind = Kind::GetSdInfo;
      break;
    case meshtastic_InterdeviceMessage_file_transfer_tag:
      out.kind = Kind::File;
      fromFile(pb->data.file_transfer, out.file);
      break;
    case meshtastic_InterdeviceMessage_sd_info_tag:
      out.kind = Kind::SdInfo;
      fromSd(pb->data.sd_info, out.sd);
      break;
    case meshtastic_InterdeviceMessage_beep_tag:
      out.kind = Kind::Beep;
      out.beep_ms = pb->data.beep;
      break;
    default:   // nmea, i2c_*, directory_listing, sd_command
      out.kind = Kind::Other;
      break;
  }
  return true;
}

}  // namespace rp2040
