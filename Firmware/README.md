# Firmware STM32

## Entorno
- Arduino IDE 2.x
- Soporte STM32: *STM32 MCU based boards* (STMicroelectronics) desde el Gestor de Placas
- Placa: STM32F405 (indicar variante exacta: ______)
- Librería IMU: BMI160 (indicar nombre y versión: ______)

## Cómo cargar
1. Abrir `stm32-gimbal/stm32-gimbal.ino`
2. Seleccionar placa y método de carga (DFU / ST-Link / Serial)
3. Compilar y cargar

## Protocolo serial
- Baudios: ______
- Formato de trama: ______ (ej. `roll,pitch,yaw,gatillo\n`)