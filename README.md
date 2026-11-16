<p align="center">
  <img src="Docs\Imagenes\Gimball.png" alt="Haptic Device Hero Image" width="100%">
</p>

<h1 align="center">Dispositivo Háptico Impulsado por Cables de 7 DoF (STM32)</h1>

<p align="center">
  <img src="https://img.shields.io/badge/Hardware-STM32F405-blue.svg" alt="Hardware">
  <img src="https://img.shields.io/badge/Sensors-BMI160%20%7C%20Magnetic%20Encoders-orange.svg" alt="Sensors">
  <img src="https://img.shields.io/badge/Teleoperation-Dobot%20CR3-green.svg" alt="Teleoperation">
  <img src="https://img.shields.io/badge/Status-In%20Development-yellow.svg" alt="Status">
</p>

> **Proyecto de Curso: Técnicas de Control**
> 
> Arquitectura mecánica, electrónica y de software para un dispositivo háptico y de captura de movimiento de **7 Grados de Libertad (DoF)** de bajo costo y alta precisión. Diseñado como interfaz maestra para teleoperar de forma bilateral un brazo robótico **Dobot CR3**.

---

## 🌟 Características Principales

El sistema está diseñado para resolver problemas clásicos de los robots paralelos impulsados por cables (CDPR), como el destensado y la fricción, dividiendo los 7 grados de libertad en dos subsistemas mecánicos:

1. **Traslación Espacial (3 DoF - X, Y, Z):** 
   - Marco cúbico accesible ($70 \times 70 \times 70\text{ cm}$).
   - Red de 4 cables anclados a un anillo de convergencia suspendido.
   - **Sistema Anti-destensado:** Uso de resortes de fuerza constante anclados en las esquinas superiores para mantener la tensión de los cables de forma mecánica en todo momento.
2. **Orientación y Gripper (4 DoF - Roll, Pitch, Yaw + Agarre):** 
   - Gimbal ergonómico suspendido en el centro de la estructura.
   - 3 DoF para la orientación de la muñeca.
   - 1 DoF para el gatillo háptico (simulación de pinza/gripper).
   - **Actuación Háptica:** 4 micro motorreductores N20 con encoders magnéticos integrados para proveer *force feedback* dinámico.

---

## 🏗️ Arquitectura del Sistema

### 1. Diseño Mecánico
<p align="center">
  <img src="Docs\Imagenes\Estructura.png" alt="Render Mecánico" width="80%">
  <br><i>(Reemplaza con un render CAD o foto de la estructura y el gimbal)</i>
</p>

El gimbal se encuentra suspendido por los 4 cables. Los motores N20 están integrados dentro del gimbal ("hacia adentro") para actuar directamente sobre las articulaciones de la mano, mientras que la odometría de traslación se mide en las esquinas de la estructura principal.

### 2. Electrónica y Control (Hardware-in-the-Loop)
<p align="center">
  <img src="Docs/Imagenes/esquema_electronico.png" alt="Esquema Electrónico" width="80%">
  <br><i>(Reemplaza con un diagrama de bloques de las conexiones)</i>
</p>

El procesamiento se realiza mediante una placa **WeAct STM32F405RGT6**, garantizando un entorno de tiempo real estricto (Hard Real-Time) sin latencias de sistema operativo.

* **Lectura de Encoders:** Uso de *Hardware Encoder Mode* en los Timers del STM32 (TIM2, TIM3, TIM5, TIM8).
* **Control de Potencia:** Drivers DRV8833 controlados por PWM (TIM1, TIM9, TIM10, TIM12) operando con una batería LiPo 2S.
* **Sensor de Actitud:** GY-BMI160 (IMU) por I2C a alta velocidad.

### 3. Teleoperación Bilateral (Software)
<p align="center">
  <img src="Docs/Imagenes/arquitectura_software.png" alt="Arquitectura de Software" width="80%">
</p>

La interfaz maestra se comunica con el esclavo (Dobot CR3) a través de una PC/ROS intermediaria:
`STM32 (USB-CDC Alta Velocidad) ➔ PC / ROS (Cinemática Inversa) ➔ TCP/IP ➔ Dobot CR3`

---

## 📂 Estructura del Repositorio

El proyecto está modularizado de la siguiente manera:

* 📁 **`/Control`**: Modelos matemáticos, cinemática directa/inversa y simulaciones de estrategias de control (compensación de fricción, control de tensión).
* 📁 **`/Docs`**: Documentación teórica, artículos del estado del arte (Scopus), presentaciones y diagramas.
* 📁 **`/Firmware`**: Código fuente C/C++ para el STM32 (lectura de IMU, decodificación de encoders por hardware, generación de PWM).
* 📁 **`/Hardware`**: 
  * `Electronica/`: Esquemáticos y PCBs.
  * `Mecanica/`: Archivos STL/STEP del marco de aluminio y las piezas del gimbal impresas en 3D.
* 📁 **`/Pruebas`**: Scripts de recolección de datos y análisis de latencia.
* 📁 **`/Software`**: Nodos de ROS y scripts de Python para la comunicación TCP/IP directa con el Dobot CR3.

---

## 🔌 Pinout y Conexiones (STM32F405)

Para evitar conflictos entre los Timers de PWM y la lectura de encoders por hardware, se utiliza la siguiente distribución estricta de pines:

| Periférico | Función | Pines STM32F405 | Notas / Asignación |
| :--- | :--- | :--- | :--- |
| **Conexión PC** | USB Nativo (D- / D+) | **PA11** / **PA12** | Envío de odometría/telemetría vía USB-CDC |
| **IMU (BMI160)** | I2C1 (SCL / SDA) | **PB6** / **PB7** | Lectura de orientación |
| **Encoder 1** | TIM2 (CH1/CH2) | **PA15** / **PB3** | Hardware Encoder Mode |
| **Encoder 2** | TIM3 (CH1/CH2) | **PA6** / **PA7** | Hardware Encoder Mode |
| **Encoder 3** | TIM5 (CH1/CH2) | **PA0** / **PA1** | Hardware Encoder Mode |
| **Encoder 4** | TIM8 (CH1/CH2) | **PC6** / **PC7** | Hardware Encoder Mode |
| **Motor 1** | TIM12 (PWM) | **PB14** / **PB15** | Control a DRV8833 |
| **Motor 2** | TIM9 (PWM) | **PA2** / **PA3** | Control a DRV8833 |
| **Motor 3** | TIM10 / TIM11 (PWM) | **PB8** / **PB9** | Control a DRV8833 |
| **Motor 4** | TIM1 (PWM) | **PA8** / **PA9** | Control a DRV8833 |

> ⚠️ **Nota de alimentación:** La lógica del STM32 y sensores funciona a **3.3V**. La potencia de los motores proviene directamente de la batería **2S (7.4V)** conectada al pin `VM` de los drivers DRV8833. Todos los GND deben estar unificados.

---

## 🚀 Cómo Empezar

1. Clona este repositorio:
   ```bash
   git clone https://github.com/TU_USUARIO/8_Dofs_Haptic_Device_STM32.git
   ```
2. Revisa la documentación de ensamblaje en `Hardware/README.md`.
3. Para compilar el código del microcontrolador, abre la carpeta `Firmware/stm32-gimbal/` utilizando STM32CubeIDE o PlatformIO.
4. Instala las dependencias de software en tu PC:
   ```bash
   pip install -r Software/dobot-interface/requeriments.txt
   ```

---
*Desarrollado para el curso de Técnicas de Control - 2026*