# Interfaz con el Dobot CR3

## Requisitos
- Python 3.10+
- Dependencias: `pip install -r requirements.txt`

## Uso
1. Conectar el gimbal por USB y verificar el puerto COM
2. Ajustar `config.py` (puerto serial, IP del Dobot)
3. Ejecutar: `python main.py`

## Notas de seguridad
- Probar primero a baja velocidad y con el área del robot despejada.
- Limitar los rangos de movimiento en el script.