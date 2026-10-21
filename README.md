# LGR100-SteamVR

**Español:** [README.es.md](README.es.md)

Experimental SteamVR driver that turns the **LG 360 VR (R100 / R105)** — an Android phone accessory made for the LG G5 (USB-C, DisplayPort Alt Mode) — into a PC VR headset on Windows: 3DOF tracking and stereoscopic rendering inside SteamVR.

> **Status: working.** Tracking is calibrated and stable, both eyes render correctly, and the headset buttons drive a manual calibration. One open bug remains: SteamVR parks the headset in standby when it sees no motion, and the mirror window then goes black.

## Status

- ✅ **3DOF tracking** — calibrated and stable (yaw drift ~0.3° over 57 s in bench testing). The driver reports the head's angular velocity, which SteamVR needs to tell a still headset from a forgotten one.
- ✅ **Panel activation and stereo rendering** — both eyes rendering, validated across several builds and confirmed visually.
- ✅ **Headset buttons** — `OK` re-measures the gyro bias and recentres the view; `BACK` re-sends the panel activation so the panels wake up without unplugging the USB-C.
- ✅ **Manual calibration** — the gyro bias is measured on demand (hold still, press `OK`) instead of being tracked continuously, which used to swallow slow rotations.
- ⚠️ **Black screen when idle** — SteamVR looks for a proximity sensor the driver does not publish, so it concludes the headset is not being worn, idles it and the mirror window goes black. It renders again as soon as something moves. See *Open issues*.
- ⚠️ **No lens distortion calibration** — none is available for this headset, so the image is presented without warp correction.

## Architecture

The driver is **one block**: it owns the HID device and talks to SteamVR directly. No OpenHMD, no second process, no intermediate API.

```
HID (VID 0x1004 / PID 0x6374)      the headset's "LGE Custom Human interface"
   -> report decoding                IMU (200 Hz), buttons, proximity
   -> sensor fusion                  orientation, gyro bias, gravity correction
   -> SteamVR plugin                 IVRDisplayComponent, projection, distortion
```

- **Display** — presented as a SteamVR `IVRDisplayComponent` (desktop window, no direct mode): a 1440×960 window with one 720×960 slot per eye. The headset has two native 960×720 panels (one per eye), so texture coordinates are rotated ±90° per eye in the distortion pass.
- **Tracking** — 3DOF orientation from the headset IMU over HID (~200 Hz), with per-axis gyro scaling, on-demand gyro bias estimation and gravity-assisted drift correction.
- **Panel control** — the panels stay dark until the driver sends the HID activation sequence ("Sleep Disable" + "VR App Start") when the device is opened. A proximity sensor keeps them on only while it is covered.
- **Hardware notes** — control interface HID `VID 0x1004 / PID 0x6374`; display enumerates as `DISPLAY\TSB8888` and exposes a single EDID mode, `1440×960 @ 55.625 Hz`.

## Things that are easy to get wrong

- **The panel has exactly one mode.** The EDID advertises `1440×960 @ 55.625 Hz` and nothing else. If SteamVR is told 60 Hz it changes the mode, which renegotiates the DP link and can knock one of the two panels out. Set `"driver_openhmd": { "displayFrequency": 57 }` in `steamvr.vrsettings` so the mode never changes.
- **The IMU stream only arrives with `hid_read_timeout(..., 0)` in a tight loop.** With a real timeout the device opens and writes fine, but not a single report is read: zero samples and a frozen orientation.
- **The panel wiring rotates each eye by 90°.** That flip belongs in the per-eye projection matrix; without it the frustum comes out with the axes swapped.
- **The HMD needs its own input profile** (`openhmd_hmd_profile.json`). Without it SteamVR cannot resolve the `openhmd_hmd` controller type, retries the binding load in a hot loop and never brings the device to a ready state.

## Open issues

- **Black screen when idle.** SteamVR asks the driver for `/user/head/proximity` (`Invalid input type button::click for path (/user/head/proximity)`) and, finding nothing, treats the headset as unworn: it enters standby and stops rendering the mirror window. Publishing input values to the action system needs the client-side API (`IVRInput`), which the server-side driver header does not provide. Both workarounds tried so far (`IServerTrackedDeviceProvider::ShouldBlockStandbyMode()` and `pauseCompositorOnStandby: false`) have no effect.
- **No lens distortion calibration** for this headset.
- **The watchdog** still carries the sample's behaviour (it wakes the system on the `Y` key) and needs a look now that the device is opened in the driver's constructor.

## Building (Windows)

Prerequisites: MSVC + CMake and SteamVR installed. The two SDK dependencies live in the project's `third_party`, next to the driver:

- `../third_party/hidapi` — hidapi headers and import library.
- `../third_party/openvr` — Valve's `openvr_driver.h` (the server-side header this plugin compiles against; the same folder holds `openvr.h` and `openvr_api.lib` for the client-side tools).

```powershell
cmake -S . -B build
cmake --build build --config Release --target driver_openhmd
```

Deploy with SteamVR fully closed (the Steam client loads the driver DLL too, so it has to be closed as well): copy `build\Release\driver_openhmd.dll` next to `hidapi.dll` in the deployed driver folder.

> Repository layout: `driver_openhmd.cpp` and `driverlog.*` are the SteamVR plugin (from [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD)); `standalone/` is the device layer that replaces OpenHMD (hidapi access, report decoding, sensor fusion, the protocol); `resources/` holds the input profiles, bindings and the driver resources. `build/` output and binaries are git-ignored.

## Credits & license

Built on the work of others:

- [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) by Christoph Haag — SteamVR plugin scaffolding, display component, resources.
- [OpenHMD](https://github.com/OpenHMD/OpenHMD) — the HID driver and sensor fusion this project started from (branch `LG-R100-rebased`); the math and the fusion are now ported into `standalone/`.
- [mSparks43/PSVR-SteamVR-openHMD](https://github.com/mSparks43/PSVR-SteamVR-openHMD) — inspiration for the standalone design.

Not affiliated with LG Electronics. This is a community reverse-engineering / hobby project.

Licensed under the **Boost Software License 1.0** — see [LICENSE](LICENSE).
