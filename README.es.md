# LGR100-SteamVR

**English:** [README.md](README.md)

Driver experimental de SteamVR que convierte el **LG 360 VR (R100 / R105)** —un accesorio para celular Android, pensado para el LG G5 (USB-C, DisplayPort Alt Mode)— en un visor de PC en Windows: tracking 3DOF y render estereoscópico dentro de SteamVR.

> **Estado: funcionando.** El tracking está calibrado y estable, los dos ojos renderizan bien y los botones del visor manejan una calibración manual. Queda un bug abierto: SteamVR pone el visor en standby cuando no ve movimiento, y la ventana espejo se queda negra.

## Estado

- ✅ **Tracking 3DOF** — calibrado y estable (deriva de yaw ~0.3° en 57 s en pruebas de banco). El driver reporta la velocidad angular de la cabeza, que es lo que SteamVR necesita para distinguir "visor quieto" de "visor olvidado".
- ✅ **Activación de paneles y render estereoscópico** — ambos ojos renderizando, validado en varios builds y confirmado a ojo.
- ✅ **Botones del visor** — `OK` vuelve a medir el bias del gyro y recentra la vista; `BACK` reenvía la activación de paneles para despertarlos sin desenchufar el USB-C.
- ✅ **Calibración manual** — el bias del gyro se mide a pedido (quedate quieto, apretá `OK`) en vez de rastrearse en continuo, que antes se comía los giros lentos.
- ⚠️ **Pantalla negra cuando está en reposo** — SteamVR busca un sensor de proximidad que el driver no publica, así que concluye que el visor no se está usando, lo idlea y la ventana espejo se queda negra. Vuelve a renderizar apenas algo se mueve. Ver *Problemas abiertos*.
- ⚠️ **Sin calibración de distorsión de lentes** — no existe para este visor, así que la imagen se presenta sin corrección de warp.

## Arquitectura

El driver es **un solo bloque**: es dueño del dispositivo HID y le habla a SteamVR directamente. Sin OpenHMD, sin un segundo proceso, sin API intermedias.

```
HID (VID 0x1004 / PID 0x6374)      el "LGE Custom Human interface" del visor
   -> decodificacion de reportes    IMU (200 Hz), botones, proximidad
   -> fusion de sensores            orientacion, bias del gyro, correccion por gravedad
   -> plugin de SteamVR             IVRDisplayComponent, proyeccion, distincion
```

- **Display** — se presenta como `IVRDisplayComponent` de SteamVR (ventana de escritorio, sin direct mode): ventana de 1440×960 con un slot de 720×960 por ojo. El visor tiene dos paneles nativos de 960×720 (uno por ojo), por lo que las coordenadas de textura se rotan ±90° por ojo en la etapa de distorsión.
- **Tracking** — orientación 3DOF desde el IMU del visor por HID (~200 Hz), con escala de gyro por eje, estimación de bias a pedido y corrección de deriva asistida por gravedad.
- **Control de paneles** — los paneles quedan apagados hasta que el driver envía la secuencia de activación por HID ("Sleep Disable" + "VR App Start") al abrir el dispositivo. Un sensor de proximidad los mantiene encendidos solo mientras está tapado.
- **Datos de hardware** — interfaz de control HID `VID 0x1004 / PID 0x6374`; el display enumera como `DISPLAY\TSB8888` y anuncia **un solo modo** en el EDID: `1440×960 @ 55.625 Hz`.

## Cosas que son fáciles de hacer mal

- **El panel tiene un único modo.** El EDID anuncia `1440×960 @ 55.625 Hz` y nada más. Si se le dice 60 Hz a SteamVR, cambia el modo, eso renegocia el link DP y puede dejar fuera a uno de los dos paneles. Dejá `"driver_openhmd": { "displayFrequency": 57 }` en `steamvr.vrsettings` para que el modo nunca cambie.
- **El stream del IMU solo llega leyendo con `hid_read_timeout(..., 0)` en loop cerrado.** Con un timeout real el dispositivo abre y escribe bien, pero no entra ni un solo reporte: cero muestras y la orientación congelada.
- **El cableado del panel rota cada ojo 90°.** Ese flip va en la matriz de proyección de cada ojo; sin él, el frustum sale con los ejes invertidos.
- **El HMD necesita su propio input profile** (`openhmd_hmd_profile.json`). Sin él SteamVR no resuelve el controller type `openhmd_hmd`, reintenta la carga de bindings en loop y el dispositivo nunca llega a estado "ready".

## Problemas abiertos

- **Pantalla negra en reposo.** SteamVR le pide al driver `/user/head/proximity` (`Invalid input type button::click for path (/user/head/proximity)`) y, al no encontrar nada, da el visor por "no usado": entra en standby y deja de renderizar la ventana espejo. Publicar valores de input en el action system requiere la API de cliente (`IVRInput`), que el header de servidor del driver no trae. Los dos workarounds probados hasta ahora (`IServerTrackedDeviceProvider::ShouldBlockStandbyMode()` y `pauseCompositorOnStandby: false`) no tienen efecto.
- **Sin calibración de distorsión de lentes** para este visor.
- **El watchdog** todavía tiene el comportamiento del sample (despierta el sistema con la tecla `Y`) y hay que revisarlo ahora que el device se abre en el constructor del driver.

## Compilación (Windows)

Requisitos: MSVC + CMake y SteamVR instalado. Las dos dependencias del SDK viven en el `third_party` del proyecto, al lado del driver:

- `../third_party/hidapi` — headers y librería de importación de hidapi.
- `../third_party/openvr` — el `openvr_driver.h` de Valve (el header de servidor contra el que compila este plugin; en la misma carpeta están `openvr.h` y `openvr_api.lib` para las tools de cliente).

```powershell
cmake -S . -B build
cmake --build build --config Release --target driver_openhmd
```

Despliegue con SteamVR completamente cerrado (el cliente de Steam también carga el DLL del driver, así que hay que cerrarlo): copiar `build\Release\driver_openhmd.dll` junto a `hidapi.dll` en la carpeta del driver desplegado.

> Estructura del repositorio: `driver_openhmd.cpp` y `driverlog.*` son el plugin de SteamVR (de [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD)); `standalone/` es la capa de dispositivo que reemplaza a OpenHMD (acceso HID, decodificación de reportes, fusión de sensores, el protocolo); `resources/` tiene los input profiles, los bindings y los recursos del driver. La salida de `build/` y los binarios están git-ignorados.

## Créditos y licencia

Construido sobre el trabajo de otros:

- [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) de Christoph Haag — estructura del plugin de SteamVR, componente de display, recursos.
- [OpenHMD](https://github.com/OpenHMD/OpenHMD) — driver HID y fusión de sensores para este visor (rama `LG-R100-rebased`); la matemática y la fusión ahora están portadas a `standalone/`.
- [mSparks43/PSVR-SteamVR-openHMD](https://github.com/mSparks43/PSVR-SteamVR-openHMD) — inspiración para el diseño standalone.

Sin relación con LG Electronics. Es un proyecto comunitario de hobby / reverse-engineering.

Licenciado bajo **Boost Software License 1.0** — ver [LICENSE](LICENSE).
