# Hardware

## Mecánica (Fusion 360)
- `mecanica/gimbal/`: gimbal de mano
- `mecanica/estructura-marco/`: marco 70x70x70 cm y esquinas impresas
- `mecanica/cables-motores/`: poleas, soportes de motor, tensores

Cada subcarpeta incluye:
- `fusion/`: archivos `.f3d` / `.f3z` editables
- `step/`: exportación STEP (formato abierto)
- `stl/`: piezas listas para imprimir
- `planos/`: PDFs con medidas

## Electrónica (EasyEDA)
- `electronica/pcb-gimbal/`: PCB del gimbal
- `electronica/pcb-motores/`: drivers de motores (futuro)

Cada PCB incluye:
- `easyeda/`: proyecto exportado (JSON de EasyEDA)
- `gerbers/`: archivos de fabricación (.zip)
- `esquematico.pdf`
- `BOM.csv`