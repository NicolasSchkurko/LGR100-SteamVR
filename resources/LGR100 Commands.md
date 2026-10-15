# LGR100 HID command reference

Everything known about the command interface of the LG 360 VR (R100 / R105), as
recovered from a firmware dump and **verified against real hardware** (tests run
on 2026-10-14 and 2026-10-15 against a retail R100). The 2026-10-15 session
re-analysed the official firmware image (`LGR100AT-00-V10d-310-XX-MAY-02-2016+0.dfu`,
extracted from the LG VR Manager APK) and added the identification commands,
the sensor self-tests, the display parameter formats and the corrected length
semantics of the second report byte.

Control interface: HID `VID 0x1004 / PID 0x6374` ("LGE Custom Human interface").
The same HID interface carries the IMU stream, the button reports, the debug
messages and the command channel.

## Report format

**Host → device (output report, 64 bytes)**

```
0x03 | opcode | ASCII command text | 0x00 padding
```

- `0x03` — the HID report ID. The interface exposes **five 64-byte report
  IDs** (`1`..`5`); commands and their replies travel on report ID 3. Observed
  input report IDs: `5` = IMU, `4` = debug text, `3` = status/command replies,
  `2` = buttons, `1` = proximity data.
- `opcode` — **the effective length of the command text**. The firmware
  terminates the received string at `byte[1]` before matching it, so:
  - `opcode < strlen` — the text is cut short and the command is *ignored*;
  - `opcode >= strlen` — the command matches (canonical value: exactly
    `strlen`).
  Verified on hardware (2026-10-15): `Sleep Disable` (13 chars) works with
  `0x0D` and is **silently ignored** with `0x0C`; `Set Brightness 100`
  (18 chars, `0x12`) works, but sent with `0x11` the firmware executes
  `Set Brightness 10`; `Accel Get XYZ` (13) only answers to `0x0D`; `Gyro Get
  XYZ` (12) answers to `0x0C` and up. Every value in the old firmware table
  (Reboot `0x06`, Gyro On `0x07`, Accel On `0x08`, GoToDload `0x09`,
  R1 Shutdown `0x0B`, Read MiniOS Result `0x12`) is simply the string length.
  The reference activator's `Sleep Disable` with `0x0C` never did anything —
  the panel came up because `VR App Start` (12 chars, `0x0C`) is correct.
- The command text is matched exactly (case sensitive). Unknown text (tested:
  `Hello World`) or an empty command is ignored silently.
- Windows: hidapi pads shorter writes to `caps.OutputReportByteLength` (64).
- **A write can fail while reads are pending, and only one process should write
  to the device.** Drain pending input reports before writing; do not run an
  external tool while the driver (SteamVR) has the device open.

**Device → host (input reports, 64 bytes)** — types observed:

| type | meaning |
| --- | --- |
| `0x05` | IMU stream, ~200 Hz. Layout: `gyro[3]` floats at bytes 1..12, `accel[3]` floats at 13..24, `tick` (u32) at 26..29. `m/s²` and raw gyro units (see `lgr100.c` scales). |
| `0x04` | ASCII debug text — command handlers' replies (`do_<Command>: ...`) and firmware logs (`BSP_...`). |
| `0x03` | Binary status frames: the `"HB"` greeting frame (sent when a client starts draining/opens the stream) and per-command status blocks (64 bytes, mostly `0xCCCC` "unset" markers). Partially decoded. |
| `0x02` | Button events. |
| `0x01` | Proximity sensor data (values change when something is in front of the sensor; see `LGR100_IRQ_UNKNOWN1` in `lgr100.c`). |
| `0x00` | Null/report padding. |
| other | `0x65`, `0xFF` also appear in the firmware's decoder; not classified here. |

The device answers a command within ~15 ms with one or more `0x04` text frames
and a `0x03` status block.

## Commands used by this driver

Sent once from `openhmd/src/drv_lgr100/lgr100.c` (`open_device`) after draining
the queue; the same pair is what `LG360VRActivator` (reference tool) sends.

| Command | Bytes | Purpose | Verified reply |
| --- | --- | --- | --- |
| `Sleep Disable` | `03 0D 53 6C ...` | Keep the device awake: disables the sleep/proximity "power down" behaviour and wakes the display path. | `04 "BSP_SM5306_On: already turn on B/L, is_vr_app_start (1), is_ba.."` + `03 01 03 30 00 80 ...` |
| `VR App Start` | `03 0C 56 52 ...` | Start VR app mode: sets the `is_vr_app_start` flag and turns the panel backlight on. Without it the panels stay dark. | `04 "BSP_SM5306_On: already turn on B/L, is_vr_app_start (1)..."` + `04 "do_VRAppStart: 0"` + `03 00 02 00 20 80 ...` |

**Driver bug (found 2026-10-15):** `lgr100.c` sends `Sleep Disable` with
`0x0C`, which is smaller than its 13-character length — the firmware truncates
the text and *ignores the command*. The byte must be `0x0D`. (The panels still
light up because `VR App Start` is sent correctly.)

`SM5306` is the display/backlight controller the firmware logs about ("BSP" =
board support).

## Full command list (firmware dump)

### Sensor stream control

| Command | Bytes prefix | Purpose |
| --- | --- | --- |
| `Accel On` | `03 08 41 63 ...` | Enable the accelerometer stream. |
| `Gyro On` | `03 07 47 79 ...` | Enable the gyroscope stream. |
| `Accel Off` | `03 0C ...` | Disable the accelerometer stream. |
| `Gyro Off` | `03 0C ...` | Disable the gyroscope stream. |
| `Accel Selftest` | `03 0E ...` | Run the BMI160 accelerometer self-test (~0.8 s of debug output). |
| `Gyro Selftest` | `03 0D ...` | Run the BMI160 gyroscope self-test (~0.8 s of debug output). |

### Single-shot sensor reads

| Command | Bytes prefix | Purpose | Verified reply |
| --- | --- | --- | --- |
| `Accel Get XYZ` | `03 0D 41 63 ...` | Read the accelerometer (averaged). Opcode `0x0D` is required; `0x0C` is ignored. | `do_AccelGetXYZ : start` → `AVERAGE 0.16.. 2.47.. -0.23..` → `do_AccelGetXYZ : fail` + `03 00 03 30 ..` |
| `Gyro Get XYZ` | `03 0C 47 79 ...` | Read the gyroscope (averaged). Accepts `0x0C` and `0x0D`. | `do_GyroGetXYZ : start` → `AVERAGE 0.00.. -0.00.. -0.00..` → `do_GyroGetXYZ : success` + `03 01 03 30 ..` |
| `Compass On` / `Compass Off` / `Compass Get XYZ` | `03 0C ...` | Magnetometer control. Not used by the driver and not tested. |

### Proximity sensor (panels off when the wearer is not detected)

| Command | Bytes prefix | Purpose | Verified |
| --- | --- | --- | --- |
| `Proximity On` / `Proximity Off` | `03 0C ...` | Enable/disable the proximity sensor. | not tested |
| `Proximity Cal` | `03 0C ...` | Calibrate the proximity sensor. | not tested |
| `Proximity Get Data` / `Get Crosstalk` / `Get Regi` | `03 0C ...` | Read proximity data / crosstalk / registers. | tested: accepted, no text reply observed |
| `Proximity Set Regi` | `03 0C ...` | Write a proximity register (needs parameters). | not tested |

### Display

| Command | Bytes prefix | Purpose | Verified |
| --- | --- | --- | --- |
| `isDisplayReady` | `03 0C ...` | Query whether the display is ready. | tested: accepted, no text reply observed |
| `Set Brightness N` | `03 12 ...` | Set panel brightness. **Parameter: a decimal number appended to the text** (e.g. `Set Brightness 100`, 18 chars → `0x12`). Values are mapped through `SM5306_blmap[]`; out-of-range values fall back to a default. | **verified 2026-10-15**: `Set Brightness 100` → `SM5306_blmap[100] = 0x2f`; `Set Brightness 4` → `out of range!! change into default value` |
| `Set Mode N` | `03 0A ...` | Set display mode. **Parameter: decimal number appended** (`Set Mode 0` = Clear, `1` = Natural, `2` = Warm). | **verified 2026-10-15**: `Set Mode 1` → `set Display Mode : 1 [0:Clear / 1:Natural / 2:Warm]` |
| `Set Backlight Control` | `03 0C ...` | Backlight control (needs a parameter). | not tested |
| `Set LCD Pattern Test` | `03 0C ...` | Put the panel into a test pattern (needs a parameter). | not tested |

### Identification (recovered 2026-10-15, verified on hardware)

All of these answer with `do_<Name>: <value>` text frames **and** a `0x03`
status frame (`03 01 <len> <ascii value...>`, e.g. `03 01 09 "REV_1.0"`).

| Command | Bytes prefix | Reply |
| --- | --- | --- |
| `Get Hwversion` | `03 0D ...` | `do_GetHWVersion : REV_1.0` |
| `Get Swversion` | `03 0D ...` | `do_GetSWVersion : LGR100AT-00-V10d-310-XX-MAY-02-2016+0` (matches the official firmware file name) |
| `Get ModelName` | `03 0D ...` | `do_GetModelName : LG-R100` |
| `Get SerialNumber` | `03 10 ...` | `do_GetSerialNumber : 123ABCD1234567` (placeholder-looking, same on this unit) |
| `Get FID` | `03 07 ...` | `do_GetFID : 12345678912345678900` (placeholder-looking) |
| `CheckSystemMode` | `03 0F ...` | accepted, **no reply observed** (tested with `0x0F` and `0x20`) |

`Get Hwversion`/`Get Swversion`/`Get ModelName`/`Get SerialNumber`/`Get FID`
are the host side of the phone app's `VRAgentServiceImpl` (com.lge.vrman);
the app builds them with `opcode = strlen` and parses the reply after the
`=`-separated token.

### Diagnostics

| Command | Bytes prefix | Purpose | Verified |
| --- | --- | --- | --- |
| `getDebugInfo` | `03 0C ...` | Ask for a debug dump. | tested: accepted, no text reply observed |
| `getAATResult` | `03 0C ...` | Ask for the AAT (auto-alignment test?) result. | tested: accepted, no text reply observed |
| `Read MiniOS Result` | `03 12 ...` | Read the MiniOS result. | tested: only a binary status frame (`03 00 42 00 20 80 ...`) |

### Dangerous — do not send casually

| Command | Bytes prefix | Effect |
| --- | --- | --- |
| `GoToDload` | `03 09 ...` | Switches the device into the DFU bootloader: the USB interface disconnects and the device re-enumerates as "LGE Download Firmware Update" (1004:6374). Recovery: physical replug (or `dfu-util -e`). This is how LG's phone app flashes firmware. |
| `Reboot` | `03 06 ...` | Reboots the firmware (dump comment: the USB device disconnects and needs a replug). **Verified 2026-10-15: no effect on this firmware unit** — no reply, no USB disconnect and the IMU tick counter keeps running. Variants `opcode 0x0C` and `R1 Reboot` also do nothing. Not usable as a software reset. |
| `R1 Shutdown` | `03 0B ...` | Shuts the device down in firmware. **Verified 2026-10-15**: it disconnects the USB interface, sends no reply/ACK, the device does **not** re-enumerate on its own and needs a physical replug (the PnP nodes stay behind with an "Unknown" status). |
| `Gyro Cal` | `03 0C ...` | **Recalibrates the gyroscope and overwrites the factory calibration.** |
| `Accel Cal` | `03 0C ...` | **Recalibrates the accelerometer and overwrites the factory calibration.** |

### Incomplete as listed

`Write Motion Result`, `Write Proximity Result`, `Write Key Result`,
`Write Earphone Result`, `Write HDMI Result`, `Write LCD Result`,
`Write Reset Result` — the dump comment notes these "seem to require additional
information, or a different send command". They are probably the host side of the
factory calibration pipeline (the phone writing calibration data back into the
firmware) and cannot be used as-is.

## Example session (captured from real hardware)

```
TX  03 0C 56 52 20 41 70 70 20 53 74 61 72 74        | "VR App Start"
RX  04 "BSP_SM5306_On: already turn on B/L, is_vr_app_start (1), is_ba.."
RX  04 "do_VRAppStart: 0"
RX  03 00 02 00 20 80 00 00 00 07 00 CC CC 08 00 CC CC ...   (status block)

TX  03 0C 47 79 72 6F 20 47 65 74 20 58 59 5A        | "Gyro Get XYZ"
RX  04 "do_GyroGetXYZ : start"
RX  04 "do_GyroGetXYZ : AVERAGE -0.0001 -0.0004 -0.0007"
RX  04 "do_GyroGetXYZ : success"
RX  03 01 03 30 00 80 00 00 00 07 00 CC CC 08 00 CC CC ...   (status block)
```

Identification and control session (2026-10-15, every byte below is
`opcode == strlen`):

```
TX  03 0D "Get Hwversion"      RX 04 "do_GetHWVersion : REV_1.0"
                               RX 03 01 09 "REV_1.0"
TX  03 0D "Get Swversion"      RX 04 "do_GetSWVersion : LGR100AT-00-V10d-310-XX-MAY-02-2016+0"
TX  03 0D "Get ModelName"      RX 04 "do_GetModelName : LG-R100"
TX  03 10 "Get SerialNumber"   RX 04 "do_GetSerialNumber : 123ABCD1234567"
TX  03 07 "Get FID"            RX 04 "do_GetFID : 12345678912345678900"
TX  03 12 "Set Brightness 100" RX 04 "SM5306_blmap[100] = 0x2f"
TX  03 0A "Set Mode 1"         RX 04 "set Display Mode : 1 [0:Clear / 1:Natural / 2:Warm]"
TX  03 0E "Accel Selftest"     RX 04 "Selftest for BMI16x starting." ... "do_AccelSelfTest : fail"
```

Notes from the log lines:

- All handler replies use the `do_<CommandName>: ...` format, spaces removed
  from the name (`do_VRAppStart`, `do_GyroGetXYZ`, `do_AccelGetXYZ`).
- The status block after each command is a 64-byte `0x03` frame whose first
  bytes vary per command; from byte 9 on it contains a table of
  `NN 00 CC CC` entries (`0xCCCC` looks like an "unset/invalid" marker). Its
  full layout is not decoded.
- `Accel Get XYZ` replied `fail` on the tested unit (gyro replied `success`);
  the raw accelerometer stream works normally, so this looks like a self-test
  result rather than a hardware fault. Not used by the driver.

## Firmware update / DFU ("dload") mode (verified 2026-10-15)

**Entering.** `GoToDload` (9 chars, opcode `0x09`) sent over the normal HID
channel makes the device reset and re-enumerate as "LGE DownLoad Firmware
Update". In that mode the USB interface exposes **HID (same vendor page,
5 report IDs × 63 bytes, i.e. a compatible interface)** *and* **DfuSe 1.1a
class requests** on control endpoint 0.

**Boot flag.** The bootloader reads one byte of flash at `0x08004000`
(sector 1): `'D'` (0x44) = forced dload (jump to the Dloader), `'I'` = ST
system ROM bootloader, anything else = boot the application (if its vector
table is valid). `GoToDload` sets `'D'`; **only a clean DFU exit clears it**.

**DFU commands** (control transfers on interface 0):

| Command | Request | Payload |
| --- | --- | --- |
| SET ADDRESS | DNLOAD, `wValue`=0 | `21 <addr LE32>` |
| ERASE page/sector | DNLOAD, `wValue`=0 | `41 <addr LE32>` |
| SET IMAGE SIZE | DNLOAD, `wValue`=0 | `81 <len LE32>` |
| WRITE block N | DNLOAD, `wValue`=N+2 | data (1024 bytes) |
| REBOOT (custom) | DNLOAD, `wValue`=0 | `82 "REBOOT"` (7 bytes) |
| **Clean exit** | DNLOAD, `wValue`=2, length 0 | — (only from `dfuIDLE`) |

(Standard DFU 1.1a GET_STATUS 0x03 / CLRSTATUS 0x04 / ABORT 0x06 as well.
UPLOAD/read was not usable in our tests: it stalls.)

**Clean exit (the missing piece).** From `dfuIDLE`, a **zero-length DNLOAD**
(`wValue`=2) moves the Dloader to state 7, clears its reboot flag, runs its
cleanup (which clears `0x08004000`) and issues a **system reset**. The
bootloader then boots the application *directly*, with its own USB
descriptors ("LGE Custom Human interface", 172-byte HID report descriptor).

**Reflash procedure** (verified with the official V10d `.dfu`; script:
`firmware/tools/dload_flash.py`):

1. ERASE `0x08020000` and ERASE `0x08040000` (LG's own pair; a third erase
   at `0x08030000` was also accepted).
2. SET IMAGE SIZE = 159716 (LG sends the whole `.dfu` length).
3. Write the system element (`0x08020000`, 134 383 bytes) in 1024-byte
   blocks with `wValue = index+2`; blocks 1..131 first, **block 0 last**
   (last partial block padded with `0xFF`).
4. Clean exit (zero-length DNLOAD) → the device reboots into the app.

Notes:

- Erasing `0x08004000` *looks* accepted but does **not** clear the boot flag
  (the Dloader's erase in dload mode only touches the system area); only the
  clean exit does.
- If a dload session is abandoned (stalled requests, aborted scripts), the
  device can end up running the app **with the Dloader's USB configuration**
  (commands/debug still work over the interrupt endpoints, but the app never
  re-enumerates its own USB). Recovery: `GoToDload` again (if needed, via the
  interrupt endpoint), then the clean exit.
- **Windows driver warning.** In DFU mode the interface comes up without a
  driver; installing **WinUSB** (Zadig, as the strfry gist suggests) replaces
  the HID driver and breaks normal access (SteamVR/hidapi stop seeing the
  device). Undo it with:
  `pnputil /enum-drivers` (find the libwdi `oemNN.inf`) then
  `pnputil /delete-driver oemNN.inf /uninstall /force` (as admin). Windows
  re-binds HIDClass on the next scan/replug.

## Reproducing these tests

Any HID write of a 64-byte output report works:

```
write: { 0x03, strlen(text), <ascii command> }   padded to 64 bytes
read:  64-byte input reports; print type 0x04 frames as text
```

Only one process may write to the device at a time: close SteamVR (so the
driver releases the HID) before sending commands from another tool, and drain
pending input reports before writing.

**Reliability warning (2026-10-15):** a long burst of writes (hundreds in a
row, as an opcode sweep does) can wedge the device's OUT endpoint: `hid_write`
then fails with `hidapi` error `0x3E5` ("Overlapped I/O operation is in
progress") for every report ID, while the IMU stream keeps running normally.
Neither draining nor re-opening the handle recovers it; only a **physical
replug** (or an elevated `pnputil /restart-device`) does. Keep test sessions
small, check the `hid_write` return value on every write, and abort on the
first `-1`.

The driver itself only needs `Sleep Disable` (`0x0D`) + `VR App Start`
(`0x0C`); everything else in this document is reference material for debugging
the hardware.
