# 🛠️ Hardware: Mecánica y Electrónica

Este directorio contiene todos los archivos de diseño físico, modelado 3D y diseño de circuitos impresos (PCBs) necesarios para construir la interfaz háptica de 7 Grados de Libertad (DoF). 

El hardware del proyecto está dividido en dos grandes ramas: **Mecánica** (estructura y gimbal) y **Electrónica** (control y potencia).

---

## ⚙️ Diseño Mecánico

El diseño tridimensional, ensamblajes y planimetría fueron desarrollados utilizando **Autodesk Fusion 360**. 

El sistema mecánico se compone de una estructura base impulsada por cables (para los 3 DoF de traslación) y un efector final tipo Gimbal (para los 4 DoF de orientación y agarre). La organización de los archivos es la siguiente:

### 📁 Rutas de la carpeta `Mecanica/`

*   📂 **`mecanica/gimbal/`**: Contiene las piezas del efector final (gimbal de mano), incluyendo los alojamientos para los 4 motores N20, el sensor IMU y el mecanismo del gatillo háptico.
*   📂 **`mecanica/estructura-marco/`**: Archivos del marco principal de 70x70x70 cm. Incluye los uniones de esquina (diseñadas para impresión 3D) que ensamblan los perfiles metálicos.
*   📂 **`mecanica/cables-motores/`**: Componentes del sistema de tracción, incluyendo soportes para los carretes, poleas de enrutamiento y los alojamientos para los resortes de fuerza constante (tensores mecánicos).

#### 🗂️ Contenido estándar de cada subcarpeta mecánica:
Dentro de cada una de las carpetas mencionadas arriba, encontrarás los archivos organizados por formato para facilitar tanto la edición como la fabricación:
*   `fusion/`: Archivos fuente editables (`.f3d` / `.f3z`) nativos de Fusion 360.
*   `step/`: Exportaciones en formato estándar `.step` (ideal para importar a SolidWorks, Inventor, etc.).
*   `stl/`: Mallas listas para importar a tu laminador (Cura, PrusaSlicer) e imprimir en 3D.
*   `planos/`: Documentos `.pdf` con las cotas, medidas principales y vistas de ensamblaje.

> **💡 Notas de Impresión 3D:** Se recomienda imprimir las piezas estructurales (esquinas del marco y soportes de poleas) en **PETG o ABS** con un relleno (infill) superior al 40% para soportar la tensión de los cables. Para el Gimbal, **PLA+** es suficiente.

---

## ⚡ Diseño Electrónico

El diseño de los esquemáticos y el rutado de las placas de circuito impreso (PCBs) se realizó utilizando **EasyEDA**. 

El hardware electrónico está diseñado para acoplar el microcontrolador STM32F405, el driver DRV8833 y gestionar las 24 líneas de conexión provenientes de los motores y encoders de forma limpia y libre de interferencias.

### 📁 Rutas de la carpeta `Electronica/`

*   📂 **`electronica/pcb-gimbal/`**: Placa principal de interconexión ubicada en el efector final. Concentra las señales del IMU GY-BMI160, los encoders y la alimentación de los motores N20 hacia el STM32.
*   📂 **`electronica/pcb-motores/`**: *[Futura Implementación]* Placa dedicada al alojamiento de los drivers de potencia (puentes H) y gestión térmica de la batería LiPo 2S.

#### 🗂️ Contenido estándar de cada subcarpeta electrónica:
Para facilitar la replicación o modificación de las placas, cada subcarpeta incluye:
*   `easyeda/`: Proyecto fuente exportado en formato `.json`, listo para ser importado de vuelta a EasyEDA.
*   `gerbers/`: Archivo `.zip` con los Gerbers estándar de fabricación. Puedes subir este archivo directamente a plataformas como JLCPCB o PCBWay.
*   `esquematico.pdf`: Diagrama del circuito en alta resolución para consulta rápida y *troubleshooting*.
*   `BOM.csv`: Lista de materiales (*Bill of Materials*) con las referencias exactas de los componentes SMD/THT necesarios para ensamblar la placa.

---

## 📸 Galería de Ensamblaje

*(Reemplaza estas etiquetas con imágenes reales de tu proyecto)*

![Render Mecánico de Fusion 360](../Docs\Imagenes\Estructura.png)
*Render del ensamblaje del Gimbal en Fusion 360.*

![PCB en EasyEDA](../Docs\Imagenes\3D_PCB_GIMBALL.png)
*Vista 3D de la PCB principal ruteada en EasyEDA.*

![Ensamblaje Físico](../Docs/Imagenes/hardware_ensamblaje_real.jpg)
*Detalle del ensamblaje físico del marco, los tensores y el enrutamiento de cables.*