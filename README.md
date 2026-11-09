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
