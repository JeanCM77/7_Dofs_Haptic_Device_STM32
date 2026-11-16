#!/usr/bin/env python3
"""
Fuerza estimada por el propio Dobot CR3 (controlador 3.5.x): corrientes y torques de las articulaciones
y fuerza en el TCP calculada por el robot a partir de las corrientes.

Lee el puerto de realimentacion 30004 (paquete de 1440 bytes cada 8 ms). Es un puerto de SOLO LECTURA:
se puede ejecutar a la vez que el puente IMU->CR3 sin interferir.

  t + Enter = tara (toma el valor actual como cero)
  q + Enter = salir
Guarda todos los paquetes en CSV_SALIDA (pon None para no guardar).
"""
import csv
import socket
import struct
import threading
import time
from collections import deque

ROBOT_IP    = "172.16.2.62"
PUERTO_FB   = 30004
CSV_SALIDA  = "fuerzas_cr3.csv"   # None = no guardar
N_TARA      = 50                  # paquetes (~0.4 s) que se promedian al hacer la tara
ALFA        = 0.2                 # filtro pasa-bajas de la fuerza mostrada (1.0 = sin filtro)
PERIODO_PRINT = 0.2               # s entre lineas impresas

TAM_PAQUETE = 1440
TEST_VALUE  = 0x0123456789ABCDEF  # el robot lo pone en el byte 48 para validar la alineacion
# Posiciones (bytes) en el paquete, segun la tabla del protocolo TCP/IP de Dobot CR
OFF = {
    "modo": 24, "t_robot": 32, "test": 48,
    "m_tgt": 384,      # MTarget:   torque objetivo de cada articulacion
    "q": 432,          # QActual:   posicion articular real (grados)
    "i_act": 528,      # IActual:   corriente real de cada articulacion
    "f_tcp": 720,      # TCPForce:  fuerza en el TCP calculada por el robot con las corrientes
    "m_act": 1120,     # MActual:   torque real de cada articulacion
}
CABECERA = (["t_local", "t_robot", "modo"]
            + [f"q{i}" for i in range(1, 7)] + [f"i{i}" for i in range(1, 7)]
            + [f"m_act{i}" for i in range(1, 7)] + [f"m_tgt{i}" for i in range(1, 7)]
            + ["Fx", "Fy", "Fz", "Mx", "My", "Mz"])


def doubles(pkt, off, n=6):
    return struct.unpack_from(f"<{n}d", pkt, off)


def parsear(pkt):
    return {
        "modo":    struct.unpack_from("<Q", pkt, OFF["modo"])[0],
        "t_robot": struct.unpack_from("<Q", pkt, OFF["t_robot"])[0],
        "q":       doubles(pkt, OFF["q"]),
        "i_act":   doubles(pkt, OFF["i_act"]),
        "m_act":   doubles(pkt, OFF["m_act"]),
        "m_tgt":   doubles(pkt, OFF["m_tgt"]),
        "f_tcp":   doubles(pkt, OFF["f_tcp"]),
    }


class Feedback(threading.Thread):
    """Hilo que lee el puerto 30004 y conserva siempre el ultimo paquete."""

    def __init__(self, ip, csv_path=None):
        super().__init__(daemon=True)
        self.ip, self.csv_path = ip, csv_path
        self.lock = threading.Lock()
        self.ultimo = None
        self.historial = deque(maxlen=N_TARA)
        self.n = 0
        self.activo = True

    def leer(self):
        with self.lock:
            return self.ultimo

    def promedio(self, clave):
        with self.lock:
            datos = [p[clave] for p in self.historial]
        if not datos:
            return [0.0] * 6
        return [sum(c) / len(datos) for c in zip(*datos)]

    def run(self):
        sock = socket.create_connection((self.ip, PUERTO_FB), timeout=3)
        sock.settimeout(1.0)
        buf = bytearray()
        f = escritor = None
        if self.csv_path:
            f = open(self.csv_path, "w", newline="")
            escritor = csv.writer(f)
            escritor.writerow(CABECERA)
        try:
            while self.activo:
                try:
                    datos = sock.recv(4096)
                except socket.timeout:
                    continue
                if not datos:
                    print("El robot cerro el puerto 30004.")
                    break
                buf += datos
                while len(buf) >= TAM_PAQUETE:
                    pkt = bytes(buf[:TAM_PAQUETE])
                    del buf[:TAM_PAQUETE]
                    if struct.unpack_from("<Q", pkt, OFF["test"])[0] != TEST_VALUE:
                        print("AVISO: paquete desalineado, reinicio el buffer.")
                        buf.clear()
                        break
                    p = parsear(pkt)
                    p["t_local"] = time.time()
                    with self.lock:
                        self.ultimo = p
                        self.historial.append(p)
                        self.n += 1
                    if escritor:
                        escritor.writerow([f"{p['t_local']:.4f}", p["t_robot"], p["modo"],
                                           *p["q"], *p["i_act"], *p["m_act"], *p["m_tgt"], *p["f_tcp"]])
        finally:
            sock.close()
            if f:
                f.close()


def main():
    fb = Feedback(ROBOT_IP, CSV_SALIDA)
    fb.start()
    t0 = time.time()
    while fb.leer() is None and time.time() - t0 < 5:
        time.sleep(0.05)
    if fb.leer() is None:
        print("No llegan datos del puerto 30004. Revisa la IP y la conexion.")
        return

    base_f = [0.0] * 6
    base_m = [0.0] * 6
    filtrada = None
    salir = {"v": False, "tara": False}

    def teclado():
        while not salir["v"]:
            c = input().strip().lower()
            if c == "t":
                salir["tara"] = True
            elif c == "q":
                salir["v"] = True

    threading.Thread(target=teclado, daemon=True).start()
    print("Leyendo el CR3. 't' + Enter = tara | 'q' + Enter = salir.")
    print("Robot QUIETO y SIN contacto para hacer la tara.\n")

    try:
        while not salir["v"]:
            if salir["tara"]:
                salir["tara"] = False
                base_f = fb.promedio("f_tcp")
                base_m = fb.promedio("m_act")
                filtrada = None
                print("Tara hecha:", [round(v, 3) for v in base_f])

            p = fb.leer()
            f = [a - b for a, b in zip(p["f_tcp"], base_f)]
            filtrada = f if filtrada is None else [ALFA * a + (1 - ALFA) * b for a, b in zip(f, filtrada)]
            m = [a - b for a, b in zip(p["m_act"], base_m)]
            print(f"modo={p['modo']:>2} | F_tcp: " + " ".join(f"{v:+7.2f}" for v in filtrada)
                  + " | torque art: " + " ".join(f"{v:+6.2f}" for v in m)
                  + " | I: " + " ".join(f"{v:+5.2f}" for v in p["i_act"]))
            time.sleep(PERIODO_PRINT)
    except KeyboardInterrupt:
        pass
    finally:
        fb.activo = False
        print(f"\nPaquetes recibidos: {fb.n}. Datos guardados en {CSV_SALIDA}.")


if __name__ == "__main__":
    main()
