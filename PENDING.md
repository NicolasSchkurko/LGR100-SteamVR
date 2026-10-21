# Pending Items — LGR100-SteamVR

Log of the project's open items, ordered from most to least urgent. Each item states its status, severity, affected files, and a concrete proposal. IDs are stable and never renumbered; resolved items move to the [Fixed](#fixed) section with their date and the reason for the change.

| # | Item | Why it is here |
| --- | --- | --- |
| 2 | `firmware/tools/dload_flash.py` referenced but not in the repository | The command reference documents the reflash procedure with a script nobody else can run |
| 3 | No lens distortion calibration | The image is presented without warp correction; no factory calibration exists for this headset |
| 4 | HID OUT endpoint can wedge under write bursts | A long burst of writes breaks `hid_write` until a physical replug; only documented, not mitigated |
| 5 | Compass stream untested; yaw drift uncorrected | 3DOF yaw drifts (~0.3° / 57 s in bench testing); the firmware exposes a magnetometer that could correct it |
| 7 | Verify the `Sleep Disable` opcode fix on hardware | `lgr100.c` now sends `0x0D`; the corrected command has not been confirmed on a real device yet |

---

## 2. Missing `dload_flash.py`

**Status:** open
**Severity:** medium (blocker for publishing the repository)
**Files:** `resources/LGR100 Commands.md`

### What happens

The reflash procedure is documented as "(script: `firmware/tools/dload_flash.py`)", but no `firmware/` directory exists in the repository. Anyone else reproducing the DFU procedure has to rewrite the script from the description.

### Proposal

Commit the script, or move the procedure inline into the command reference (the command table in that document is already complete enough to reimplement it).

## 3. No lens distortion calibration

**Status:** open
**Severity:** medium
**Files:** `driver_openhmd.cpp`

### What happens

No distortion calibration is available for this headset, so the image is presented without warp correction. Geometry is usable but not optically matched to the lenses.

### Proposal

Derive a calibration (e.g. fit a lens distortion model from test-pattern photos through the lenses) or adapt published parameters for comparable 2016 phone-VR optics. The display path already exposes per-eye distortion hooks.

## 4. HID OUT endpoint wedging

**Status:** open (documented, not mitigated)
**Severity:** medium
**Files:** `resources/LGR100 Commands.md`, `openhmd/src/drv_lgr100/lgr100.c`

### What happens

A long burst of HID writes (hundreds in a row, as an opcode sweep does) can wedge the device's OUT endpoint: every subsequent `hid_write` fails with `0x3E5` ("Overlapped I/O operation is in progress") while the IMU stream keeps running. Neither draining nor reopening the handle recovers it; only a physical replug does.

### Proposal

Harden the probing tooling: check the `hid_write` return value on every write, abort on the first `-1`, and keep sessions small. The driver itself only sends two commands at open, so normal operation is low-risk; this item is about the test tool.

## 5. Compass stream untested; yaw drift uncorrected

**Status:** open
**Severity:** low
**Files:** `resources/LGR100 Commands.md`, `openhmd/src/drv_lgr100/lgr100.c`

### What happens

3DOF orientation drifts in yaw (~0.3° over 57 s in bench testing) because gravity only constrains pitch and roll. The firmware exposes a magnetometer (`Compass On` / `Compass Get XYZ`), listed in the command reference but never tested.

### Proposal

Test the compass stream, decode its units, and evaluate it as a slow yaw reference (complementary filter). If the magnetometer is too noisy or absent on retail units, keep the current drift figures documented as a known limitation.

## 7. Verify the `Sleep Disable` opcode fix on hardware

**Status:** open (verification pending)
**Severity:** low
**Files:** `openhmd/src/drv_lgr100/lgr100.c`

### What happens

The command reference found that `Sleep Disable` was sent with opcode `0x0C` (< 13-char length), so the firmware silently truncated and ignored it. The code now sends `0x0D`, but the corrected command has not been confirmed on a real device yet.

### Proposal

On the next hardware session, capture the debug stream after opening the device and confirm the expected `BSP_SM5306_On: already turn on B/L` reply for `Sleep Disable` in addition to the `VR App Start` one.

---

## Fixed

### Tracking calibration corrected — 2026-10-14

`openhmd/src/drv_lgr100/lgr100.c`: per-axis gyro scales (1.25 pitch / 1.47 roll) over-integrated by ~19 %/~41 %, producing large pose errors after fast head movements. All axes now converge to a gravity-verified factor of 1.05. Added startup gyro-bias estimation that only updates while the device is genuinely still. The fusion loop now uses the measured tick rate (1/374325 s) instead of a nominal 200 Hz.

### Diagnostic CSV logging disabled — 2026-10-14

Enabling the per-sample CSV dump on the update thread crashed `vrserver` with a write fault inside `ucrtbase`. The logger remains in the code behind `LGR100_TRACKING_LOG 0` for dedicated diagnostic runs only.

### `Sleep Disable` opcode corrected — 2026-10-15

`lgr100.c` sent the command with a length byte of `0x0C`; the firmware truncates to that length, so the command never matched. Corrected to `0x0D`. Hardware verification is item 7.

### HID protocol and DFU documented — 2026-10-15

`resources/LGR100 Commands.md`: full command reference recovered from the firmware dump and verified against a retail R100 — report layouts, the opcode-as-length semantics (with truncation proof), identification commands, sensor self-tests, display parameters, the DFU boot flag at `0x08004000`, the zero-length-DNLOAD clean exit, and a reflash procedure verified with the official V10d `.dfu`.

### Intermittent black left eye resolved — 2026-10-20

`driver_openhmd.cpp`: stereo rendering now works across builds; the earlier left-eye dropout in real scenes (e.g. Steam Home) no longer reproduces after the standalone driver took over panel activation and frame submission. Both eyes validated visually.

### Phase 1: standalone driver completed — 2026-10-20

Repository-wide: OpenHMD dependency dropped. The driver now talks to the HID directly via vendored hidapi (`standalone/`), running its own sensor fusion, report decoding and protocol; gyro scales, bias estimation and tick timing ported from the OpenHMD branch. HID is reconnected automatically after a replug. OpenVR/hidapi SDKs vendored under `third_party/`; Valve's sample icons and broken references removed.

### Headset buttons and on-demand gyro calibration — 2026-10-20

`standalone/lgr100_device.cpp`: `OK` re-measures gyro bias and recentres; `BACK` re-sends panel activation. Manual calibration replaces continuous bias tracking, which used to swallow slow rotations.


---

## Before publishing the repository

- [ ] Commit `firmware/tools/dload_flash.py` (or move the reflash instructions inline into the command reference).
- [x] License review: Boost 1.0 for this project and the vendored OpenHMD subset; `openvr/openvr_driver.h` keeps Valve's copyright header.
- [x] Personal data scan: no local paths, usernames or credentials committed.
- [ ] Confirm the README status wording is up to date on the day of publishing (it is currently honest work-in-progress).
- [x] Add repository topics (e.g. `steamvr`, `openvr`, `openhmd`, `virtual-reality`, `reverse-engineering`, `hid`).
- [x] Optional: resolve the black left eye first; publishing with the bug is acceptable while the status section stays accurate.

---

*Last updated: 2026-10-21*
