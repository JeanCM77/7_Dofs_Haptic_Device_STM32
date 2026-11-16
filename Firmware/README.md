# 💻 Firmware: STM32F405 y Control en Tiempo Real

Este directorio contiene el código fuente que se ejecuta en el microcontrolador **WeAct STM32F405RGT6**. Este componente actúa como el "cerebro" en tiempo real de la interfaz háptica, encargado de leer los sensores a alta velocidad y controlar la resistencia (force feedback) de los motores.

## 🛠️ Entorno de Desarrollo

El proyecto está programado utilizando el entorno de Arduino, aprovechando el soporte oficial de STMicroelectronics para placas STM32.

### Requisitos del Software:
* **IDE:** [Arduino IDE 2.x](https://www.arduino.cc/en/software)
* **Gestor de Placas (Board Manager):** 
  * Instalar el paquete: `STM32 MCU based boards` (por STMicroelectronics).
* **Configuración de la Placa:**
  * **Board:** `Generic STM32F4 series`
  * **Board Part Number:** `STM32F405RGx` (Correspondiente a la WeAct Mini Core).
  * **USB Support:** `CDC (generic 'Serial' supersede U(S)ART)` *<- Importante para la comunicación rápida.*

### Dependencias y Librerías:
Para compilar el proyecto, asegúrate de instalar las siguientes librerías desde el *Library Manager* de Arduino:
* **Librería IMU:** `DFRobot_BMI160` (Versión 1.0.0 o superior) - Utilizada para leer el acelerómetro y giroscopio por I2C.
* *(Si se utilizan otras librerías para filtros PID o Kalman, listarlas aquí)*.

## 🚀 Cómo Cargar el Firmware

El código principal del gimbal se encuentra en la carpeta `stm32-gimbal`. Para cargarlo en tu STM32, sigue estos pasos:

1. Conecta la placa STM32F405 a tu PC.
2. Abre el archivo principal: `stm32-gimbal/Orientacion_BIM_ENC_STM.ino` en Arduino IDE.
3. Selecciona la placa configurada (`STM32F405RGx`).
4. Selecciona el **Método de Carga (Upload Method):**
   * **ST-Link:** Si usas un programador ST-Link V2 externo (Recomendado para debug).
   * **DFU / STM32CubeProgrammer:** Si lo conectas directamente por USB mientras presionas el botón `BOOT0` de la placa.
   * **Serial:** Si usas un conversor USB-TTL genérico.
5. Haz clic en **Verificar** y luego en **Subir (Upload)**.

## 📡 Protocolo de Comunicación Serial

El STM32 y el PC (donde corre ROS o los scripts de control del Dobot CR3) se comunican mediante un puerto serial virtual a través de la conexión USB nativa (pines PA11 / PA12). 

* **Puerto:** Virtual COM Port (USB-CDC)
* **Baud Rate (Baudios):** `115200` *(Puede subirse a `921600` si se requiere menor latencia en el control háptico).*
* **Frecuencia de Actualización:** ~100 Hz a 500 Hz (configurable en el código).

### Formato de la Trama de Datos (STM32 ➔ PC)

El microcontrolador envía los datos de orientación y del gatillo en texto plano, separados por comas (CSV) y terminados con un salto de línea (`\n`). 

**Estructura:**
```text
<Roll>,<Pitch>,<Yaw>,<Posicion_Gatillo>\n
```

**Ejemplo de lectura:**
```text
45.20,-12.50,90.15,1024
45.25,-12.48,90.20,1030
```
*(Donde `Roll`, `Pitch` y `Yaw` están en grados, y `Posicion_Gatillo` son los pulsos o el ángulo leídos por el encoder hardware del TIM8).*

### Formato de Comandos (PC ➔ STM32)

Para la retroalimentación de fuerza (*force feedback*), el PC puede enviar un comando de resistencia que ajusta el PWM de los motores a través de los drivers DRV8833.

**Estructura (Ejemplo):**
```text
F,<Fuerza_Gatillo>,<Fuerza_Roll>,<Fuerza_Pitch>,<Fuerza_Yaw>\n
```

## 🧠 Notas sobre la Arquitectura del Código

* **Hard Real-Time:** La lectura de los 4 encoders magnéticos se realiza mediante los **Timers en Hardware** del STM32 (TIM2, TIM3, TIM5 y TIM8). Esto significa que el conteo de pulsos no interrumpe el ciclo de ejecución principal (`loop()`), logrando una latencia de 0 en la lectura.
* **Control PWM:** El driver DRV8833 recibe las señales PWM usando los Timers 1, 9, 10 y 12. Se ha implementado un límite de ciclo de trabajo (aprox. 70-75%) por software para proteger los motores de 6V cuando se utiliza la batería LiPo 2S (7.4V - 8.4V).