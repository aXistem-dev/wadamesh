// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// The microSD card in the RP2040's slot as an Arduino fs::FS, for map tiles:
// a file-level FSImpl over SdCore and the RP2040 bridge link, with no FatFs on
// this side. What it carries is what the tile code needs: open "r" (read, seek,
// size) and "w" (sequential write, committed on close), exists, mkdir and
// remove. "a", "r+", rename, rmdir and directory listing are not supported.
//
// Threads: every operation runs under one recursive mutex, which also covers
// SdCore's copy of reply data out of the link. Calls from the task that ran
// indicatorSdBegin() (Arduino's loop task, which runs the UI) take that mutex
// with a 20 ms timeout, wait out at most 2 BUSY replies and a 120 ms reply
// timeout, and after any such failure fail at once for a while (300 ms,
// doubling with each failure in a row up to 4.8 s), so the UI never stalls on
// the card; the map draws the tile as missing and comes back to it. Other
// tasks (the core-0 tile fetcher) wait normally.

#include <FS.h>
#include <stdint.h>

class IndicatorSdFs : public fs::FS {
public:
  IndicatorSdFs();
};

extern IndicatorSdFs indicatorSd;

// Starts the RP2040 bridge (two pings) and reads the card info once; logs both.
// Call once from setup(), after IndicatorIo::begin(): the calling task is the
// one treated as the UI thread. False when the bridge did not answer, and the
// whole feature then stays off.
bool indicatorSdBegin();

// The bridge answered at boot. Without it the feature is off and the map
// behaves as on a board without a card slot. No I/O.
bool indicatorSdAvailable();

// The bridge answered at boot and the last card info said a card is present.
// No I/O.
bool indicatorSdReady();

// False once the bridge turned out to have no MKDIR (stock firmware): the card
// is then read-only and cannot cache tiles. True until a write was tried.
bool indicatorSdWritable();

// From the UI loop while the map uses the card: polls the card info every 5 s
// (sooner after a failed write or while a card mounts, up to 30 s apart while
// the bridge does not answer) and notes card changes.
void indicatorSdPoll(uint32_t now_ms);

// True once after the card appeared, went, was swapped or the RP2040
// restarted: the map's tile source has to be re-checked.
bool indicatorSdTakeChange();

// True once after a UI-thread read failed for a passing reason (link busy,
// card busy, lost reply) and the fail-fast window is over: tiles drawn as
// missing are worth another try.
bool indicatorSdTakeRetry();

// True while UI-thread reads fail at once after a passing failure.
bool indicatorSdReadBlocked();

// Asks for a card-info poll at the next indicatorSdPoll(); any task.
void indicatorSdPollSoon();

// Once a minute: "[indicator] sd: tiles=<n> avg=<n>ms/tile" for the UI-thread
// tile reads since the last line.
void indicatorSdLogStats(uint32_t now_ms);
