#!/usr/bin/env python3
"""
Puente IMU (STM32 por USB) -> Dobot CR3 (TCP/IP).  Empieza con UN eje y agrega los demas.

  e + Enter = enganchar (la referencia es la posicion actual de IMU y robot)
  s + Enter = soltar (el robot se queda donde esta)
  q + Enter = salir

Requisitos: pip install pyserial.  Cierra el Monitor Serie de Arduino (usa el mismo puerto).
Orden de uso: 1) SIMULAR = True (no mueve nada)  2) SIMULAR = False con limites pequenos.
"""
import math
import re
import socket
import threading
import time

import serial

# ============================ CONFIGURACION ============================
ROBOT_IP   = "172.16.2.90"
PUERTO_STM = "COM14"          # revisa el puerto en el Administrador de dispositivos
BAUDIOS    = 115200

SIMULAR      = True          # True = solo imprime lo que enviaria. NO mueve el robot.
MODO         = "JOINT"       # "JOINT" -> ServoJ (articulaciones) | "POSE" -> ServoP (Rx,Ry,Rz)
SPEED_FACTOR = 10            # % de velocidad global
SEND_HZ      = 20            # Dobot recomienda >= 30 ms entre comandos Servo (max ~33 Hz)

# Eje de la IMU -> destino en el robot. Activa UN eje y ve agregando.
#   JOINT: destino = indice de articulacion (0=J1 ... 5=J6)
#   POSE : destino = indice de [x,y,z,rx,ry,rz] (3=Rx, 4=Ry, 5=Rz)
DESTINO = {
    "JOINT": {"R": 5, "P": 4, "Y": 3},
    "POSE":  {"R": 3, "P": 4, "Y": 5},
}
EJES = {
    "R": {"activo": True,  "ganancia": 0.5, "signo": 1.0},   # Roll
    "P": {"activo": True, "ganancia": 0.5, "signo": 1.0},   # Pitch
    "Y": {"activo": True, "ganancia": 0.5, "signo": 1.0},   # Yaw (deriva: sin magnetometro)
}

ZONA_MUERTA  = 1.0           # grados de la IMU que se ignoran (filtra ruido y temblor)
LIMITE_DEG   = 10.0          # maximo desplazamiento respecto a la referencia (grados)
PASO_MAX_DEG = 1.0           # maximo cambio por envio (limita la velocidad angular)
TIMEOUT_IMU  = 0.3           # s sin datos validos de la IMU -> se suelta solo
# ======================================================================


def envolver(a):
    return (a + 180.0) % 360.0 - 180.0


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


def zona_muerta(d, z):
    return 0.0 if abs(d) <= z else d - math.copysign(z, d)


class Robot:
    def __init__(self, ip):
        self.dash = socket.create_connection((ip, 29999), timeout=5)
        self.move = socket.create_connection((ip, 30003), timeout=3)
        self.dash.settimeout(5.0)  # <-- CAMBIA ESTO DE 1.0 a 5.0
        self.move.settimeout(0.05)

    def _tx(self, sock, cmd):
        sock.sendall(cmd.encode("utf-8"))
        try:
            return sock.recv(1024).decode("utf-8", errors="ignore").strip()
        except socket.timeout:
            return ""

    def dashboard(self, cmd, mostrar=True):
        r = self._tx(self.dash, cmd)
        if mostrar:
            print(f"  {cmd} -> {r}")
        return r

    @staticmethod
    def _numeros(resp):
        m = re.search(r"\{([^}]*)\}", resp)
        if not m or not m.group(1):
            raise RuntimeError(f"Respuesta inesperada del robot: {resp!r}")
        return [float(v) for v in m.group(1).split(",")]

    def angulos(self):
        return self._numeros(self.dashboard("GetAngle()", False))[:6]

    def pose(self):
        return self._numeros(self.dashboard("GetPose()", False))[:6]

    def modo_robot(self):
        try:
            return int(self._numeros(self.dashboard("RobotMode()", False))[0])
        except Exception:
            return -1

    def servo(self, comando, valores):
        cmd = f"{comando}(" + ",".join(f"{v:.3f}" for v in valores) + ")"
        r = self._tx(self.move, cmd)
        if r and not r.startswith("0"):
            print(f"  AVISO robot: {r}")
            return False
        return True

    def cerrar(self):
        for s in (self.dash, self.move):
            try:
                s.close()
            except OSError:
                pass


estado = {"pedir": False, "enganchado": False, "salir": False}


def teclado():
    while not estado["salir"]:
        c = input().strip().lower()
        if c == "e":
            estado["pedir"] = True
        elif c == "s":
            estado["enganchado"] = False
            print("Soltado: el robot se queda donde esta.")
        elif c == "q":
            estado["salir"] = True


def leer_imu(ser):
    """Parsea lineas tipo  P:1.2<TAB>R:3.4<TAB>Y:5.6<TAB>E1:0 ..."""
    linea = ser.readline().decode("utf-8", errors="ignore").strip()
    d = dict(re.findall(r"(\w+):(-?\d+(?:\.\d+)?)", linea))
    if all(k in d for k in ("P", "R", "Y")):
        return {k: float(d[k]) for k in ("P", "R", "Y")}
    return None


def main():
    modo = MODO.upper()
    destino = DESTINO[modo]
    activos = [k for k, c in EJES.items() if c["activo"]]
    print(f"Modo {modo} | SIMULAR={SIMULAR} | ejes activos: {activos}")

    robot = Robot(ROBOT_IP)
    if not SIMULAR:
        robot.dashboard("ClearError()")
        robot.dashboard("EnableRobot()")
        time.sleep(0.5)
        robot.dashboard(f"SpeedFactor({SPEED_FACTOR})")

    ser = serial.Serial(PUERTO_STM, BAUDIOS, timeout=0.05)
    threading.Thread(target=teclado, daemon=True).start()
    print("Listo. 'e' = enganchar, 's' = soltar, 'q' = salir.")

    imu, ref_imu, ref, cmd = None, None, None, None
    t_rx = time.time()
    t_tx = t_print = t_modo = 0.0

    try:
        while not estado["salir"]:
            nuevo = leer_imu(ser)
            if nuevo:
                imu, t_rx = nuevo, time.time()

            if estado["pedir"] and imu:
                estado["pedir"] = False
                ref_imu = dict(imu)
                ref = robot.angulos() if modo == "JOINT" else robot.pose()
                cmd = list(ref)
                estado["enganchado"] = True
                print(f"Enganchado. Referencia robot: {[round(v, 2) for v in ref]}")

            if not estado["enganchado"]:
                continue

            if time.time() - t_rx > TIMEOUT_IMU:
                estado["enganchado"] = False
                print("Sin datos de la IMU: soltado por seguridad.")
                continue

            if not SIMULAR and time.time() - t_modo > 0.5:
                t_modo = time.time()
                if robot.modo_robot() == 9:       # 9 = alarma
                    estado["enganchado"] = False
                    print("El robot esta en ALARMA: soltado. Revisa y usa ClearError().")
                    continue

            if time.time() - t_tx >= 1.0 / SEND_HZ:
                t_tx = time.time()
                deltas = {}
                for k in activos:
                    cfg = EJES[k]
                    d = zona_muerta(envolver(imu[k] - ref_imu[k]), ZONA_MUERTA)
                    desp = limitar(cfg["signo"] * cfg["ganancia"] * d, -LIMITE_DEG, LIMITE_DEG)
                    i = destino[k]
                    objetivo = ref[i] + desp
                    paso = limitar(objetivo - cmd[i], -PASO_MAX_DEG, PASO_MAX_DEG)
                    cmd[i] += paso
                    deltas[k] = (d, cmd[i])

                if modo == "JOINT":
                    envio = cmd
                else:
                    envio = cmd[:3] + [envolver(a) for a in cmd[3:]]

                if not SIMULAR:
                    if not robot.servo("ServoJ" if modo == "JOINT" else "ServoP", envio):
                        estado["enganchado"] = False
                        print("El robot rechazo el comando: soltado.")
                        continue

                if time.time() - t_print > 0.25:
                    t_print = time.time()
                    texto = "  ".join(
                        f"{k}: IMU {d:+6.2f} deg -> destino[{destino[k]}] = {v:+8.3f}"
                        for k, (d, v) in deltas.items()
                    )
                    print(("[SIM] " if SIMULAR else "") + texto)
    finally:
        print("Cerrando.")
        ser.close()
        robot.cerrar()


if __name__ == "__main__":
    main()
