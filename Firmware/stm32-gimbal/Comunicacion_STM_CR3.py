#!/usr/bin/env python3
"""
Puente IMU (STM32 por USB) -> Dobot CR3 (TCP/IP), con posicion HOME al iniciar.

Flujo:
  1) Conecta, habilita el robot y (si IR_A_HOME) lo lleva a HOME_JOINTS con un movimiento lento.
  2) Espera a que lo enganches:  e + Enter  (la referencia es la posicion actual de IMU y robot)
  3) El robot sigue los cambios de la IMU respecto a ese enganche.

  e + Enter = enganchar | s + Enter = soltar | q + Enter = salir
Requisitos: pip install pyserial.  Cierra el Monitor Serie de Arduino (usa el mismo puerto).
"""
import math
import re
import socket
import threading
import time

import serial

# ============================ CONFIGURACION ============================
ROBOT_IP   = "172.16.2.24"
PUERTO_STM = "COM14"
BAUDIOS    = 115200

SIMULAR      = False         # True = solo imprime lo que enviaria. NO mueve el robot (ni va a HOME).
MODO         = "JOINT"       # "JOINT" -> ServoJ (articulaciones) | "POSE" -> ServoP (Rx,Ry,Rz)
SPEED_FACTOR = 50            # % de velocidad global
SEND_HZ      = 30            # Dobot recomienda >= 30 ms entre comandos Servo (max ~33 Hz)

# ---- Posicion HOME al iniciar ----
IR_A_HOME      = True
HOME_JOINTS    = (90, 0, 0, 0, -90, 0)   # J1..J6 en grados
VEL_HOME       = 30                   # % de SpeedJ y AccJ del movimiento a HOME
CONFIRMAR_HOME = True                 # pide Enter antes de moverse (area despejada)
AUTO_ENGANCHAR = False                # True = engancha solo al llegar a HOME (la IMU debe estar en neutro)
TOLERANCIA_HOME = 1.0                 # grados: si alguna articulacion queda mas lejos, avisa

# Eje de la IMU -> destino en el robot.
#   JOINT: destino = indice de articulacion (0=J1 ... 5=J6)
#   POSE : destino = indice de [x,y,z,rx,ry,rz] (3=Rx, 4=Ry, 5=Rz)
DESTINO = {
    "JOINT": {"R": 5, "P": 3, "Y": 4},
    "POSE":  {"R": 3, "P": 3, "Y": 4},
}
EJES = {
    "R": {"activo": True, "ganancia": 1, "signo": 1.0},   # Roll
    "P": {"activo": True, "ganancia": 1, "signo": 1.0},   # Pitch
    "Y": {"activo": True, "ganancia": 1, "signo": 1.0},   # Yaw (deriva: sin magnetometro)
}

ZONA_MUERTA  = 2.0           # grados de la IMU que se ignoran (filtra ruido y temblor)
LIMITE_DEG   = 30.0          # maximo desplazamiento respecto a la referencia (grados)
PASO_MAX_DEG = 3.0           # maximo cambio por envio (limita la velocidad angular)
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
        self.dash.settimeout(5.0)
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

    def ir_a_home(self, joints, vel):
        """Movimiento articular lento a 'joints'. Bloquea hasta terminar (Sync). True si fue bien."""
        j = ",".join(f"{v:.3f}" for v in joints)
        self.move.settimeout(2.0)
        r = self._tx(self.move, f"JointMovJ({j},SpeedJ={vel},AccJ={vel})")
        print(f"  JointMovJ -> {r}")
        if r and not r.startswith("0"):
            self.move.settimeout(0.05)
            return False
        self.move.settimeout(120.0)          # el movimiento puede tardar
        r = self._tx(self.move, "Sync()")
        print(f"  Sync -> {r}")
        self.move.settimeout(0.05)
        return bool(r) and r.startswith("0")

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


estado = {"pedir": False, "enganchado": False, "salir": False, "calibrar": False}


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
        elif c == "c":
            estado["calibrar"] = True


def leer_imu(ser):
    """
    Parsea líneas tipo:
    P:1.2\tR:3.4\tY:5.6\tE1:0 ...
    
    Vacía el buffer serie y se queda con la última línea recibida.
    """
    linea = None

    # Descarta todas las líneas pendientes y conserva la última
    while ser.in_waiting:
        linea = ser.readline()

    # Si no había nada pendiente, espera una línea normalmente
    if linea is None:
        linea = ser.readline()

    linea = linea.decode("utf-8", errors="ignore").strip()

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

        if IR_A_HOME:
            try:
                print("Angulos actuales:", [round(v, 2) for v in robot.angulos()])
                if CONFIRMAR_HOME:
                    input(f"\nEl robot ira a HOME {HOME_JOINTS} a {VEL_HOME}%.\n"
                          "Area despejada? Enter para continuar (Ctrl+C cancela)... ")
                print("Yendo a HOME...")
                if not robot.ir_a_home(HOME_JOINTS, VEL_HOME):
                    print("No se pudo completar el movimiento a HOME. Revisa el robot (ClearError / alarma).")
                    robot.cerrar()
                    return
                ang = robot.angulos()
                print("Angulos en HOME:", [round(v, 2) for v in ang])
                if any(abs(a - h) > TOLERANCIA_HOME for a, h in zip(ang, HOME_JOINTS)):
                    print("AVISO: el robot no quedo exactamente en HOME (fuera de tolerancia).")
            except KeyboardInterrupt:
                print("\nCancelado: ResetRobot()")
                robot.dashboard("ResetRobot()")
                robot.cerrar()
                return

    ser = serial.Serial(PUERTO_STM, BAUDIOS, timeout=0.05)
    threading.Thread(target=teclado, daemon=True).start()
    if AUTO_ENGANCHAR:
        estado["pedir"] = True
        print("Listo. Enganche automatico con la primera lectura de la IMU. 's' = soltar, 'q' = salir.")
    else:
        print("Listo. Pon la IMU en neutro y pulsa 'e' para enganchar. 's' = soltar, 'q' = salir.")

    imu, ref_imu, ref, cmd = None, None, None, None
    t_rx = time.time()
    t_tx = t_print = t_modo = 0.0

    try:
        while not estado["salir"]:
            # --- NUEVO BLOQUE DE CALIBRACIÓN ---
            if estado["calibrar"]:
                estado["calibrar"] = False
                estado["enganchado"] = False  # Soltamos el robot por seguridad
                print("\n[!] Calibrando IMU. ¡NO MUEVAS EL SENSOR!")
                ser.write(b'c')       # Enviar comando al STM32
                time.sleep(2.5)       # Esperar a que el STM32 termine (~1.5s de delay interno)
                ser.reset_input_buffer() # Limpiar basura acumulada en el buffer
                print("IMU calibrada y estabilizada. Presiona 'e' para volver a enganchar.")
            # -----------------------------------

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