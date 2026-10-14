# LGR100-SteamVR

**English:** [README.md](README.md)

Driver experimental de SteamVR que convierte el **LG 360 VR (R100 / R105)** —un accesorio para celular Android, pensado para el LG G5 (USB-C, DisplayPort Alt Mode)— en un visor de PC en Windows: tracking 3DOF y render estereoscópico dentro de SteamVR.

> **Estado: en desarrollo.** El tracking está calibrado y funcionando; el render estereoscópico funciona en ambos ojos en la mayoría de los builds, con un ojo izquierdo negro intermitente que sigue en investigación.

## Estado

- ✅ **Tracking 3DOF** — calibrado y estable (deriva de yaw ~0.3° en 57 s en pruebas de banco).
- ✅ **Activación de paneles y render estereoscópico** — ambos ojos renderizando, validado en varios builds.
- ⚠️ **Ojo izquierdo negro intermitente** — bug abierto. Los logs confirman que ambos ojos reciben frustums y calibración de distorsión idénticos y válidos, así que la matemática de proyección no es la causa; la evidencia apunta a la presentación/tiempos del compositor (se rompe al cargar una escena real, como Steam Home).
- ⚠️ **Sin calibración de distorsión de lentes** — no existe para este visor, así que la imagen se presenta sin corrección de warp.

## Arquitectura (actual)

- **Display** — se presenta como `IVRDisplayComponent` de SteamVR (ventana de escritorio, sin direct mode): ventana de 1440×960 con un slot de 720×960 por ojo. El visor tiene dos paneles nativos de 960×720 (uno por ojo), por lo que las coordenadas de textura se rotan ±90° por ojo en la etapa de distorsión.
- **Tracking** — orientación 3DOF desde el IMU del visor por HID (~200 Hz), con escala de gyro por eje, estimación de bias y corrección de deriva asistida por gravedad.
- **Control de paneles** — los paneles quedan apagados hasta que el driver envía la secuencia de activación por HID ("Sleep Disable" + "VR App Start") al abrir el dispositivo. Un sensor de proximidad los mantiene encendidos solo mientras está tapado.
- **Datos de hardware** — interfaz de control HID `VID 0x1004 / PID 0x6374` ("LGE Custom Human interface"); el display enumera como `DISPLAY\TSB8888`.

## Roadmap

1. **Fase 0 — actual**: driver funcionando sobre [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) y [OpenHMD](https://github.com/OpenHMD/OpenHMD) (rama `LG-R100-rebased`).
2. **Fase 1 — standalone**: eliminar la dependencia de OpenHMD, siguiendo el enfoque de [mSparks43/PSVR-SteamVR-openHMD](https://github.com/mSparks43/PSVR-SteamVR-openHMD): el plugin habla HID directo con **hidapi** y hace su **propia fusión de sensores**, portando las escalas de gyro, la estimación de bias y el timing de tick ya calibrados. Mantiene el `IVRDisplayComponent` / la distorsión ya validados. Huella chica (~600–900 líneas), sin dependencias frágiles.
3. **Fase 2 — pulido**: arreglar el ojo izquierdo negro intermitente; revisar el manejo de la distorsión de lentes.

## Compilación (Windows)

Requisitos: MSVC + CMake, SteamVR instalado, y las fuentes de OpenHMD (rama `LG-R100-rebased`) en `subprojects/openhmd`.

```powershell
cmake -S . -B build
cmake --build build --config Release --target driver_openhmd
```

Despliegue con SteamVR completamente cerrado: copiar `build\Release\driver_openhmd.dll` junto a `hidapi.dll` en la carpeta del driver desplegado.

> El código fuente del driver se va a importar a este repositorio en commits siguientes; este commit inicial es el esqueleto del proyecto (README + licencia).

## Créditos y licencia

Construido sobre el trabajo de otros:

- [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) de Christoph Haag — estructura del plugin de SteamVR, componente de display, recursos.
- [OpenHMD](https://github.com/OpenHMD/OpenHMD) — driver HID y fusión de sensores para este visor (rama `LG-R100-rebased`).
- [mSparks43/PSVR-SteamVR-openHMD](https://github.com/mSparks43/PSVR-SteamVR-openHMD) — inspiración para el diseño standalone.

Sin relación con LG Electronics. Es un proyecto comunitario de hobby / reverse-engineering.

Licenciado bajo **Boost Software License 1.0** — ver [LICENSE](LICENSE).
