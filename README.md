# CDPR Haptic 7-DoF

Dispositivo háptico y de captura de movimiento de **7 grados de libertad**, de bajo costo y alta precisión, basado en un robot paralelo de cables (CDPR). Se usa como líder para teleoperar un brazo robótico **Dobot CR3**.

> Proyecto desarrollado en la Universidad Nacional de Colombia, Sede La Paz.

<!-- Agrega aquí una foto o GIF del gimbal -->
<!-- ![Gimbal](docs/imagenes/gimbal.jpg) -->

## Descripción

- **Traslación XYZ (3 DoF):** 4 cables con motores y encoders, dentro de un marco de 70x70x70 cm.
- **Orientación y gripper (4 DoF):** gimbal de mano con IMU y encoders, y gatillo háptico para la pinza.
- **Objetivo:** teleoperar el Dobot CR3 y evaluar técnicas de control moderno (LQR, observadores, MPC, entre otras).

## Arquitectura


## Estado del proyecto

| Módulo | Estado |
|---|---|
| Gimbal: estructura mecánica | ✅ Listo |
| Gimbal: electrónica y sensórica | ✅ Listo |
| Firmware STM32 (orientación) | 🔧 En proceso |
| Envío de orientación al Dobot (Python) | 🔧 En proceso |
| Estructura de suspensión por cables | 🔧 En proceso |
| Motores y control de tensión de cables | ⏳ Pendiente |
| Retroalimentación háptica | ⏳ Pendiente |
| Control moderno (LQR / observadores / MPC) | ⏳ Pendiente |

## Estructura del repositorio

| Carpeta | Contenido |
|---|---|
| `docs/` | Bitácora, estado del arte, imágenes, presentaciones |
| `hardware/mecanica/` | Modelos CAD de Fusion 360 (gimbal, marco, cables) |
| `hardware/electronica/` | Proyectos de PCB en EasyEDA, gerbers, datasheets |
| `firmware/` | Código del STM32 (Arduino IDE) |
| `software/` | Interfaz con el Dobot, simulaciones, herramientas |
| `control/` | Modelos matemáticos y parámetros |
| `pruebas/` | Datos y resultados experimentales |

## Hardware principal

- Microcontrolador **STM32F405**
- IMU **BMI160**
- Motores N20 con encoder (gimbal)
- Brazo robótico **Dobot CR3**

## Cómo empezar

1. **Firmware:** ver [`firmware/README.md`](Firmware/README.md)
2. **Software (Python):** ver [`software/dobot-interface/README.md`](software/dobot-interface/README.md)
3. **CAD y PCB:** ver [`hardware/README.md`](hardware/README.md)

## Bitácora

Cada avance queda registrado en [`docs/bitacora/`](docs/bitacora/) y resumido en [`CHANGELOG.md`](CHANGELOG.md).

## Autores

- Jean Mejía Jiménez
- Angelly Ríos Rincón

## Licencia

Software: MIT. Hardware y documentación: CC BY-NC-SA 4.0. Ver [`LICENSE`](LICENSE).
