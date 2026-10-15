# LGR100-SteamVR

**Español:** [README.es.md](README.es.md)

Experimental SteamVR driver that turns the **LG 360 VR (R100 / R105)** — an Android phone accessory made for the LG G5 (USB-C, DisplayPort Alt Mode) — into a PC VR headset on Windows: 3DOF tracking and stereoscopic rendering inside SteamVR.

> **Status: work in progress.** Tracking is calibrated and working; stereoscopic rendering works in both eyes in most builds, with an intermittent black left eye still under investigation.

## Status

- ✅ **3DOF tracking** — calibrated and stable (yaw drift ~0.3° over 57 s in bench testing).
- ✅ **Panel activation and stereo rendering** — both eyes rendering, validated across several builds.
- ⚠️ **Intermittent black left eye** — open bug. Logs confirm both eyes get identical, valid frusta and distortion calibration, so the projection math is not the cause; evidence points to presentation/compositor timing (it breaks when a real scene such as Steam Home loads).
- ⚠️ **No lens distortion calibration** — none is available for this headset, so the image is presented without warp correction.

## Architecture (current)

- **Display** — presented as a SteamVR `IVRDisplayComponent` (desktop window, no direct mode): a 1440×960 window with one 720×960 slot per eye. The headset has two native 960×720 panels (one per eye), so texture coordinates are rotated ±90° per eye in the distortion pass.
- **Tracking** — 3DOF orientation from the headset IMU over HID (~200 Hz), with per-axis gyro scaling, gyro bias estimation and gravity-assisted drift correction.
- **Panel control** — the panels stay dark until the driver sends the HID activation sequence ("Sleep Disable" + "VR App Start") when the device is opened. A proximity sensor keeps them on only while it is covered.
- **Hardware notes** — control interface HID `VID 0x1004 / PID 0x6374` ("LGE Custom Human interface"); display enumerates as `DISPLAY\TSB8888`.

## Roadmap

1. **Phase 0 — current**: working driver on top of [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) and [OpenHMD](https://github.com/OpenHMD/OpenHMD) (branch `LG-R100-rebased`).
2. **Phase 1 — standalone**: remove the OpenHMD dependency, following the approach of [mSparks43/PSVR-SteamVR-openHMD](https://github.com/mSparks43/PSVR-SteamVR-openHMD): the plugin talks HID directly via **hidapi** and runs its **own sensor fusion**, porting the already-calibrated gyro scales, bias estimation and tick timing. Keeps the validated `IVRDisplayComponent` / distortion path. Small footprint (~600–900 LOC), no fragile dependencies.
3. **Phase 2 — polish**: fix the intermittent black left eye; revisit lens distortion handling.

## Building (Windows)

Prerequisites: MSVC + CMake, SteamVR installed, and a [hidapi](https://github.com/libusb/hidapi) build (headers + import library). The OpenHMD subset the driver needs is vendored under `openhmd/`; nothing else is required.

```powershell
cmake -S . -B build `
  -DHIDAPI_INCLUDE_DIR=<hidapi>\include `
  -DHIDAPI_LIBRARY=<hidapi>\x64\hidapi.lib
cmake --build build --config Release --target driver_openhmd
```

Deploy with SteamVR fully closed: copy `build\Release\driver_openhmd.dll` next to `hidapi.dll` in the deployed driver folder.

> Repository layout: `driver_openhmd.cpp`, `driverlog.*` and `ohmd_config.h` are the SteamVR plugin (from [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD)); `openhmd/` is the frozen OpenHMD subset (Phase 0) and `openvr/` vendors Valve's `openvr_driver.h`. `build/` output and binaries are git-ignored.

## Credits & license

Built on the work of others:

- [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) by Christoph Haag — SteamVR plugin scaffolding, display component, resources.
- [OpenHMD](https://github.com/OpenHMD/OpenHMD) — HID driver and sensor fusion for this headset (branch `LG-R100-rebased`).
- [mSparks43/PSVR-SteamVR-openHMD](https://github.com/mSparks43/PSVR-SteamVR-openHMD) — inspiration for the standalone design.

Not affiliated with LG Electronics. This is a community reverse-engineering / hobby project.

Licensed under the **Boost Software License 1.0** — see [LICENSE](LICENSE).
