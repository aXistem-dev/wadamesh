# Seeed SenseCAP Indicator D1L port

PlatformIO environment:

```text
sensecap_indicator_companion_radio_touch
```

The SenseCAP Indicator D1L is an ESP32-S3 (8 MB flash, 8 MB octal PSRAM) with a 4-inch 480x480 RGB panel behind an ST7701S, an FT6336 capacitive touch controller and an SX1262. A TCA9535 I/O expander carries the radio chip select, reset, BUSY and DIO1 lines as well as the display chip select and reset and the touch reset and interrupt. A separate RP2040 co-processor owns the buzzer, microSD, Grove connector and (on the D1Pro) the sensors; this build uses its microSD, for map tiles, and its buzzer, for notification beeps, through the RP2040's UART bridge firmware. The D1Pro shares the ESP32 side of the board and is expected to run this image, but it is untested; the D1L is the reference target.

The board is USB-powered and has no battery. `CAP_BATTERY` is 0, so the status bar has no battery glyph and the Battery settings page (history, calibrate, battery saver) is hidden. Power off is hidden.

## Hardware

| Function | Pins / device |
|---|---|
| I2C | SDA 39, SCL 40, 400 kHz |
| TCA9535 expander | 0x20; /INT on GPIO 42 (active low, falling edge) |
| Display | ST7701S, 16-bit RGB, 480x480; data D0..D15 = GPIO 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0; DE 18, VSYNC 17, HSYNC 16, PCLK 21; init commands over a bit-banged 3-wire SPI on SCLK 41, MOSI 48 (shared with the radio SPI) |
| Backlight | GPIO 45, LEDC channel 7, 12 kHz |
| Touch | FT6336 at 0x48 |
| SX1262 | SPI FSPI at 4 MHz: SCLK 41, MISO 47, MOSI 48; NSS, RESET, BUSY and DIO1 on the expander; DIO2 drives the RF switch; DIO3 supplies the TCXO (2.4 V) when the strap selects it; current limit 140 mA, boosted RX gain |
| User button | GPIO 38 |
| RP2040 | held out of reset by expander P1.0; bridge link on UART2 at 2 Mbaud, ESP TX GPIO 19 / RX GPIO 20 (the USB pads, released to the UART) |

Expander map (TCA9535, port 0 then port 1):

| Pin | Function |
|---|---|
| P0.0 | LoRa NSS (out) |
| P0.1 | LoRa RESET (out) |
| P0.2 | LoRa BUSY (in) |
| P0.3 | LoRa DIO1 (in) |
| P0.4 | LCD CS (out) |
| P0.5 | LCD RESET (out) |
| P0.6 | touch INT (in) |
| P0.7 | touch RESET (out) |
| P1.0 | RP2040 RESET (driven high) |
| P1.3 | radio strap (in; high = TCXO fitted) |

Every other port-1 bit keeps the state it had at boot. The output and configuration shadows are seeded from the chip, never from fixed defaults, so state left by a warm reset survives. A failed I2C write rolls both shadows back.

The radio pins are virtual numbers that the HAL maps to the expander: `P_LORA_NSS=0x40`, `P_LORA_RESET=0x41`, `P_LORA_BUSY=0x42`, `P_LORA_DIO_1=0x43`.

## Boot sequence

`IndicatorBoard::begin()` runs `ESP32Board::begin()`, then `IndicatorIo::begin()` under the shared I2C bus lock: 400 kHz clock, expander probe and shadow seed, NSS/CS/RESET outputs driven high, the four inputs configured, and a five-sample read of the radio strap (a majority decides the TCXO selection, so a single glitch cannot flip it). `[indicator] expander seeded out=.. cfg=..` logs the registers as the previous firmware or a warm reset left them, `[indicator] expander out=..` the result. A failure names its step: seed read, output config, input config or strap read.

Then `indicatorSdBegin()` starts the RP2040 bridge: two pings of 500 ms at most, then one card-info request. `[indicator] rp2040: bridge v2` and `[indicator] sd: card=<present|none|busy|?> size=<MB> free=<MB>` report the result: `busy` while the RP2040 is still mounting the card, `?` when the card-info request got no answer, and `?` for a size or free space not known yet. The same line is logged on each card change. Without a version-2 pong it logs `[indicator] rp2040: no bridge (<reason>)` and the map behaves as on a board without a card slot.

`display.begin()` follows: radio NSS high, LCD CS high, LCD RESET low for 10 ms and high again with a 120 ms settle, then with the bus lock held CS low, the LovyanGFX panel init, CS high. Holding the lock keeps any other task from moving the radio NSS while the init bit-bangs the shared GPIO 41/48. The panel init restores those two pins afterwards, and nothing writes panel commands after init. The backlight starts at about 63 % over a black frame. If the expander is not ready, `_lcd.init()` fails (`[indicator] display: lcd.init failed`) or an expander write during the sequence fails, `display.begin()` leaves the backlight off and returns false. The backlight then stays off: the UI still sends brightness and wake requests to the display, and the driver ignores them until the panel has come up.

`radio_init()` runs after the display, so the radio's FSPI claims GPIO 41/47/48 once LovyanGFX has released them.

## Radio

The SX1262 sits behind the expander, so RadioLib runs on `IndicatorRadioHal`, a subclass of `ArduinoHal`:

- Pins with the expander flag (`0x40` and up) go through the TCA9535; every other pin goes straight to the Arduino pin and interrupt functions, as in `ArduinoHal`. `RADIOLIB_NC` is not an expander pin.
- NSS toggles around every SPI transfer, each edge one expander write. RESET adds a 20 ms settle after its rising edge.
- Every expander access holds the shared I2C bus lock, which is a recursive mutex with priority inheritance.

Interrupt dispatch: the expander's /INT output is wired to GPIO 42. Its ISR (in IRAM) only notifies the `ind_dio1` task (3072-byte stack, high priority, core 1). That task reads port 0 under the bus lock and never touches RadioLib or SPI. `Dio1Edge` watches bit 3 of every port-0 read and fires the RadioLib receive callback exactly once per 0-to-1 transition. This holds whichever reader sees the edge: the dispatch task, or a BUSY poll inside an SPI transfer (that read clears /INT, so the edge is still delivered from there). A DIO1 level that stays high while another input changes (touch INT, BUSY) does not fire again. The task also wakes every 50 ms as a net for a /INT that was already low when the ISR attached.

/INT wake log: several port-1 inputs float and the expander has no pull-ups, so /INT can toggle continuously, and every toggle costs an I2C read at high priority. The dispatch task counts ISR notifications and 50 ms timeouts and logs `[indicator] /INT wakes=<n>/10s timeouts=<n>` every 10 s for the first minute, then only while the ISR rate is above 500/s, at most once a minute.

Polled backstop: `MESH_RADIO_DIO1_POLLED=1` makes the radio wrapper poll the SX1262 IRQ status in the receive path, so a lost expander edge cannot stall RX.

TCXO strap: P1.3 high means a TCXO is fitted. `indicatorTcxoVoltage()` returns 2.4 V in that case and 0 V (crystal) otherwise; `SX126X_DIO3_TCXO_VOLTAGE` calls it at radio init. The flag is quoted in `platformio.ini` so that SCons' shell does not choke on the parentheses. When the radio does not start with the strap's setting, `radio_init()` tries the other one (0 V if the strap said TCXO, 2.4 V if it said crystal) before it gives up, since `std_init` only falls back from TCXO to 0 V by itself; `main.cpp`'s one retry runs both again. `[indicator] radio tcxo requested=..` logs the request and `[indicator] radio up, tcxo used=..` the voltage the radio came up with.

`IndicatorBoard` overrides `getIRQGpio()` (42) and `sleep()` (a 1 ms delay) so the base class never uses the virtual expander pin for GPIO wake.

## Display

LovyanGFX 1.2.27 `Panel_ST7701` on `Bus_RGB`, 480x480, `use_psram` framebuffer, panel init list 1 as in the Meshtastic device-ui reference (vertical flip and horizontal flip), rotation fixed at 0 (UI rotation is `LV_DISP_ROT_NONE`; the keyboard-rotate controls are compiled out).

Timings: horizontal 8 / 50 / 10 (pulse / back porch / front porch), vertical 8 / 20 / 10, polarities 0, DE idle high, PCLK idle low and rising-edge active.

Pixel clock: `INDICATOR_PCLK_HZ=12000000` (12 MHz, 240 MHz / 20, about 42 Hz with these timings). The RGB bus reads the PSRAM framebuffer continuously and this core has no bounce buffers, so the limit is PSRAM bandwidth under load rather than the panel: 12 MHz keeps the picture stable under a busy UI with Wi-Fi traffic, around 15 MHz the picture can shift briefly, and from about 17 MHz it flickers. The value is a build flag, so trying another clock needs no source change.

LVGL draw buffer: a 480x24 band (23040 bytes), allocated in internal DMA RAM with PSRAM as the fallback. The other CPU-fed boards draw into a larger band in PSRAM; here the panel already scans its frame buffer out of PSRAM, so the band stays internal and the flush is a single copy into PSRAM. LVGL is compiled at -O2 (`custom_lvgl_o2`); its IRAM option (`custom_lvgl_iram`) is left off: it costs +5.9 KB IRAM, and with the 23 KB internal band that would leave less internal heap than the T-Deck's.

Home screen: the widget grid needs nothing board-specific. It sizes its cells from the shorter side of the panel, so the square panel gets about four columns by four rows at Normal, three by three at Large and two by two at Huge, and it keeps the layout saved for landscape, since the panel is not taller than wide. Other screens keep their portrait layout.

Brightness: the UI slider drives `Light_PWM`; 0 switches the backlight off (duty 0). Screen sleep writes 0 and wake restores the slider level.

Colour byte order: pixels are written as on the other LovyanGFX boards (`setSwapBytes(true)` with `LV_COLOR_16_SWAP` 0). Red, green and blue render correctly on the panel.

## Touch

FT6336 at 0x48 on the shared I2C bus, polled by the `indicator_touch` task on core 0 (3072-byte stack). Registers: status plus first point at 0x02 (5 bytes), chip id at 0xA3 (0x64 expected, logged, not fatal), vendor id at 0xA8 (logged). The reset pulse goes through the expander (P0.7) and runs on the first probe only. If the controller does not answer, the UI retries the probe at most every 2 s, and the failure is logged once.

Rotation: the controller reports both axes mirrored relative to the panel, so `x = 479 - raw_x` and `y = 479 - raw_y`, clamped to 0..479. The raw point is kept for the calibration screen. Gesture thresholds (40 and 16 px, 12 ms) are shared with the Wio Tracker L2 touch code.

## microSD (via the RP2040)

The microSD slot is wired to the RP2040, not to the ESP32, so the card is reached through the RP2040's bridge firmware over UART2. `IndicatorSdFs` (`indicatorSd`) is a file-level `fs::FS` over the bridge's file requests (`IndicatorSdCore.h`, `Rp2040Uart.h`). The card carries map tiles only.

Link: frames of `0x94 0xC3`, a big-endian 16-bit length and an `InterdeviceMessage` of at most 4666 bytes (`Rp2040Link.h`). The messages are coded with nanopb (`nanopb/Nanopb @ 0.4.92`, in this env's `lib_deps` only) and the `proto/interdevice.pb.{h,c}` it generated from Meshtastic's `interdevice.proto` plus the MKDIR operation; the header of those files says how to regenerate them. Replies are paired with requests by id.

Bridge firmware: the RP2040 needs an indicator_rp2040 build that answers ping version 2 and implements MKDIR (file_transfer operation 4); the Wi-Fi tile cache needs MKDIR to create its directories. Meshtastic's stock bridge build has no MKDIR and answers it with IO_ERROR "unknown operation" (or a nack). With that firmware the card is read-only from the first tile fetch on: offline packs still read, there is no Wi-Fi tile cache, and no tiles are downloaded.

Card layout:

| Path | Content |
|---|---|
| `/maps/osm/{z}/{x}/{y}.png` | offline tile packs, read with "Tiles from SD card" on |
| `/tiles/{z}/{x}/{y}.jpg` | Wi-Fi tile cache, on the card either way (the partition table has no tiles partition) |

Chat history, contacts, settings and every other data store stay on internal flash (SPIFFS and NVS). The bridge has no rename, positioned write or directory listing, so the data store, the file manager and card formatting do not use the card.

Speed: a tile read typically costs around 140 ms; the timing log below reports the figure for the card in use. Reads go out in 4096-byte chunks with an 8-chunk read-ahead, up to 3 requests in flight; a negative cache of 256 entries (60 s each) answers repeated misses without asking the card.

Writes: requests that write (POST, PUT, DELETE, MKDIR) wait up to 5 s for their reply (`kSdWriteReplyTimeoutMs`); GETs wait 500 ms. The bridge remounts the card each time it re-checks it: after a GET answered NOT_FOUND, and every 5 s once the card has been idle for 1 s. After a remount SdFat searches for a free cluster from the start of the FAT, so the first write of a new file can take about 3 s on a card whose free space starts far into the FAT; later writes take tens to a few hundred milliseconds. A write that fails for good removes its partial file.

Threads: one recursive mutex covers every operation. The UI thread takes it with a 20 ms timeout, waits out at most 2 BUSY replies and 120 ms per reply, then fails fast for 300 ms (doubling with each failure in a row, up to 4.8 s) and redraws the missing tiles afterwards. The tile fetcher on core 0 waits normally (30 BUSY retries of 100 ms, and the reply timeouts above).

Absent bridge or card:

- No bridge: without a version-2 pong at boot the feature is off, `[indicator] rp2040: no bridge (<reason>)` is logged, the map has no "Tiles from SD card" row, downloads no tiles (there is nowhere to keep them) and says that the RP2040 firmware has no microSD bridge.
- No card: `sd: card=none`; the map has no tile store and shows its storage notice. A card inserted later is picked up by the card-info poll.
- Card pulled during a read: the read fails within one reply timeout and the tile shows as missing; the UI does not freeze. BUSY replies while the RP2040 mounts a card are never cached as "not found", so tiles return once the card is back.

Card changes: there is no card-detect line, so the UI loop polls the card info every 5 s while the map uses the card (1 s while a card mounts; 5 s, then 30 s apart while the bridge does not answer). A new, removed or swapped card, or an RP2040 restart, re-checks the tile source.

Timing log: `[indicator] sd: tiles=<n> avg=<n>ms/tile` once a minute while the map tab is open on the card (UI-thread card time per tile read, misses included).

## Buzzer (via the RP2040)

The buzzer is wired to the RP2040 too. The bridge's `beep` message sounds it for a given number of milliseconds at one fixed pitch and gets no reply, so `indicatorBeep()` (`IndicatorBuzzer.h`) sends it without waiting for an answer. A map tile write holds the link for a whole file; a beep that cannot get the link within half a second is skipped. The notification sounds differ by count: one 70 ms beep for a message, two for a direct message, three for an @-mention. They follow the usual Sound settings (master switch, per-event switches, channel mutes, Do Not Disturb); "Loud alerts" is hidden, since the pitch cannot change, and so is the volume. Meshtastic's stock bridge firmware handles `beep` as well. Without a bridge nothing sounds.

## Flash layout and image size

Partition table `variants/sensecap_indicator/partitions_sensecap_indicator.csv`, 8 MB:

| Name | Offset | Size |
|---|---|---|
| nvs | 0x9000 | 0x5000 |
| otadata | 0xe000 | 0x2000 |
| app0 | 0x10000 | 0x380000 |
| app1 | 0x390000 | 0x380000 |
| spiffs | 0x710000 | 0xE0000 |
| coredump | 0x7F0000 | 0x10000 |

The image takes about 3.4 MB (roughly 92 %) of the 3670016-byte (0x380000) app slot.

## Not in this port

- RP2040 peripherals: the Grove I2C connector and the D1Pro sensors are not used.
- The microSD carries map tiles only. The bridge has no rename, positioned write or directory listing, so the data store, the file manager, card formatting and the other SD features stay off; chat history, contacts and settings stay on internal flash.
- Deep sleep and power off: nothing can wake the board, so Power off is hidden and deep sleep is not offered.
- OTA release artifacts: this is a development target; no web-flasher or OTA release image is produced.
- Battery reporting: there is no battery; `getBattMilliVolts()` returns 0.

## Validation status

Exercised on a D1L: boot after a cold power-on, a reset and a reboot from the UI; colours and orientation; touch; the UI at Normal, Large and Huge; the Constellation screensaver and a radar widget on Home at 12 MHz; the user button; channel and direct messages sent and received with ACKs, including a two-hour direct-message soak with no reset; DM timestamps on the receiving node; the idle `/INT` wake rate; the RP2040 bridge at boot, offline map tiles from the microSD and the card pulled and put back while the map is open; the beep patterns for a message, a direct message and an @-mention. Not yet exercised: the D1Pro, the first boot after Meshtastic or the Seeed stock firmware, every backlight level, the spectrum scan, the Wi-Fi tile cache writes and a long map session.
