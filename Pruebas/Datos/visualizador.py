#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualizador en tiempo real: Sensor (IMU/STM32)  vs  Dobot CR3  +  margen de error.

Qué hace
  * Lee la IMU del gimbal por USB-CDC (trama "P:..\tR:..\tY:..", igual que Comunicacion_STM_CR3.py).
  * Lee los ángulos articulares reales del Dobot CR3 por el puerto de realimentación 30004 (solo lectura).
  * (Opcional) Hace de puente IMU -> CR3 (ServoJ), con los mismos parámetros y lógica que
    Comunicacion_STM_CR3.py, para que el robot siga al sensor mientras se grafica.
  * Grafica en vivo: ángulo del sensor, ángulo del Dobot y error entre ambos, mostrando la
    proporción matemática sensor:Dobot.
  * Al finalizar (botón "Finalizar y guardar" o cerrar la ventana) guarda automáticamente
        Pruebas/Datos/image_AAAA-MM-DD_HH-MM-SS.png
        Pruebas/Datos/data_AAAA-MM-DD_HH-MM-SS.csv

Definiciones
  * Ángulo del sensor  = IMU(t) - IMU(referencia)              [grados]
  * Ángulo del Dobot   = Junta_real(t) - Junta_real(referencia) [grados]
  * Dobot esperado     = signo * ganancia * ángulo del sensor   (proporción sensor:Dobot = 1 : ganancia)
  * Error              = Ángulo del Dobot - Dobot esperado      [grados]

Uso
  python visualizador_tiempo_real.py            # hardware real (puente IMU->CR3 + gráficas)
  python visualizador_tiempo_real.py --monitor  # solo observar (no envía comandos al robot)
  python visualizador_tiempo_real.py --demo     # datos simulados, sin hardware

Dependencias:  pip install customtkinter matplotlib numpy pyserial
IMPORTANTE: el puerto COM de la IMU solo lo puede abrir un programa a la vez. Cierra el Monitor Serie de
Arduino y NO ejecutes Comunicacion_STM_CR3.py a la vez: este script ya incluye el puente.
"""
import argparse
import csv
import math
import queue
import re
import socket
import struct
import threading
import time
import traceback
from bisect import bisect_left
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

import numpy as np
import customtkinter as ctk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ============================ CONFIGURACION ============================
# --- Mantener sincronizado con Comunicacion_STM_CR3.py ---
ROBOT_IP     = "172.16.2.62"
PUERTO_STM   = "COM14"
BAUDIOS      = 115200
SPEED_FACTOR = 30            # % de velocidad global del robot
SEND_HZ      = 30            # frecuencia de envío de ServoJ
HOME_JOINTS  = (90, 0, 0, 0, -90, 0)
VEL_HOME     = 30            # % de SpeedJ y AccJ en el movimiento a HOME

# Eje de la IMU -> índice de articulación del robot (0=J1 ... 5=J6). Solo modo JOINT.
DESTINO = {"R": 5, "P": 3, "Y": 4}
EJES = {
    "R": {"activo": True, "ganancia": 0.5, "signo": 1.0},   # Roll
    "P": {"activo": True, "ganancia": 0.5, "signo": 1.0},   # Pitch
    "Y": {"activo": True, "ganancia": 0.5, "signo": 1.0},   # Yaw
}
ZONA_MUERTA  = 2.0          # grados de la IMU que se ignoran
LIMITE_DEG   = 30.0         # máximo desplazamiento respecto a la referencia
PASO_MAX_DEG = 3.0          # máximo cambio por envío
TIMEOUT_IMU  = 0.3          # s sin datos de la IMU -> se suelta solo

# --- Visualización / exportación ---
VENTANA_S    = 30.0         # segundos visibles en la gráfica en vivo
GUI_HZ       = 30           # muestreo de la GUI (puntos por segundo guardados)
PLOT_CADA    = 2            # redibuja la gráfica cada N muestreos
RUTA_SALIDA  = None         # None = autodetectar <proyecto>/Pruebas/Datos ; o pon una ruta propia
# ======================================================================

KEYS   = ("R", "P", "Y")
NOMBRE = {"R": "Roll", "P": "Pitch", "Y": "Yaw"}
COLOR  = {"R": "#ff6b81", "P": "#2ee6c5", "Y": "#ffc857"}

# Paleta de la interfaz
BG, PANEL, PANEL2 = "#0b1020", "#131a2e", "#1a2340"
GRID, TXT, TXT_DIM = "#27304f", "#e6ebf7", "#8e9abb"
ACCENT, OK, BAD, WARN = "#5b8cff", "#2ee6a6", "#ff5d73", "#ffc857"

# Protocolo de realimentación del Dobot CR (puerto 30004)
PUERTO_FB, TAM_PAQUETE, TEST_VALUE = 30004, 1440, 0x0123456789ABCDEF
OFF_MODO, OFF_TEST, OFF_Q = 24, 48, 432


# ----------------------------------------------------------------------
# Utilidades matemáticas
# ----------------------------------------------------------------------
def envolver(a):
    return (a + 180.0) % 360.0 - 180.0


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


def zona_muerta(d, z):
    return 0.0 if abs(d) <= z else d - math.copysign(z, d)


def ganancia_total(k):
    """Factor sensor -> Dobot del eje k (signo * ganancia)."""
    return EJES[k]["signo"] * EJES[k]["ganancia"]


def fmt_ganancia(g):
    return f"{g:+.3g}".replace("+", "") if g < 0 else f"{g:.3g}"


def texto_proporcion():
    """Proporción matemática sensor:Dobot, p. ej. 'Sensor : Dobot = 1 : 0.5  ( Δ Dobot = 0.5 · Δ Sensor )'."""
    gs = {round(ganancia_total(k), 6) for k in KEYS}
    if len(gs) == 1:
        g = ganancia_total("R")
        return f"Proporción  Sensor : Dobot  =  1 : {fmt_ganancia(g)}   (Δ Dobot = {fmt_ganancia(g)} · Δ Sensor)"
    return "Proporción Sensor : Dobot  →  " + " | ".join(
        f"{NOMBRE[k]} 1 : {fmt_ganancia(ganancia_total(k))}" for k in KEYS)


def carpeta_salida():
    """<proyecto>/Pruebas/Datos, buscándola a partir de la ubicación del script."""
    if RUTA_SALIDA:
        return Path(RUTA_SALIDA)
    aqui = Path(__file__).resolve().parent
    for p in [aqui, *aqui.parents]:
        if p.name.lower() == "pruebas":
            return p / "Datos"
        if p.name.startswith("7_Dofs_Haptic_Device_STM32"):
            return p / "Pruebas" / "Datos"
    return Path.cwd() / "Pruebas" / "Datos"


# ----------------------------------------------------------------------
# Fuentes de datos
# ----------------------------------------------------------------------
class LectorIMU(threading.Thread):
    """Lee la IMU del STM32 y conserva siempre la última lectura. Reintenta si se desconecta."""

    def __init__(self, puerto, baud, log):
        super().__init__(daemon=True)
        self.puerto, self.baud, self.log = puerto, baud, log
        self.lock = threading.Lock()
        self.dato, self.t_rx = None, 0.0
        self.conectado, self.activo = False, True

    def latest(self):
        with self.lock:
            return (dict(self.dato), time.time() - self.t_rx) if self.dato else (None, 1e9)

    def run(self):
        import serial
        avisado = False
        while self.activo:
            try:
                ser = serial.Serial(self.puerto, self.baud, timeout=0.2)
            except Exception as e:
                self.conectado = False
                if not avisado:
                    self.log(f"IMU: no se pudo abrir {self.puerto} ({e}). Reintentando…")
                    avisado = True
                time.sleep(1.0)
                continue
            avisado, self.conectado = False, True
            self.log(f"IMU conectada en {self.puerto}.")
            try:
                while self.activo:
                    linea = ser.readline().decode("utf-8", errors="ignore").strip()
                    d = dict(re.findall(r"(\w+):(-?\d+(?:\.\d+)?)", linea))
                    if all(k in d for k in KEYS):
                        with self.lock:
                            self.dato = {k: float(d[k]) for k in KEYS}
                            self.t_rx = time.time()
            except Exception as e:
                self.log(f"IMU: conexión perdida ({e}).")
            finally:
                self.conectado = False
                try:
                    ser.close()
                except Exception:
                    pass


class LectorDobot(threading.Thread):
    """Lee QActual (grados de J1..J6) del puerto 30004 del CR3. Puerto de solo lectura."""

    def __init__(self, ip, log):
        super().__init__(daemon=True)
        self.ip, self.log = ip, log
        self.lock = threading.Lock()
        self.q, self.modo, self.t_rx = None, 0, 0.0
        self.conectado, self.activo = False, True

    def latest(self):
        with self.lock:
            return (list(self.q), time.time() - self.t_rx) if self.q else (None, 1e9)

    def run(self):
        avisado = False
        while self.activo:
            try:
                sock = socket.create_connection((self.ip, PUERTO_FB), timeout=3)
            except Exception as e:
                self.conectado = False
                if not avisado:
                    self.log(f"Dobot: sin conexión a {self.ip}:{PUERTO_FB} ({e}). Reintentando…")
                    avisado = True
                time.sleep(2.0)
                continue
            avisado, self.conectado = False, True
            self.log(f"Dobot conectado ({self.ip}).")
            sock.settimeout(1.0)
            buf = bytearray()
            try:
                while self.activo:
                    try:
                        datos = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not datos:
                        break
                    buf += datos
                    while len(buf) >= TAM_PAQUETE:
                        pkt = bytes(buf[:TAM_PAQUETE])
                        del buf[:TAM_PAQUETE]
                        if struct.unpack_from("<Q", pkt, OFF_TEST)[0] != TEST_VALUE:
                            buf.clear()
                            break
                        with self.lock:
                            self.q = list(struct.unpack_from("<6d", pkt, OFF_Q))
                            self.modo = struct.unpack_from("<Q", pkt, OFF_MODO)[0]
                            self.t_rx = time.time()
            except Exception as e:
                self.log(f"Dobot: conexión perdida ({e}).")
            finally:
                self.conectado = False
                sock.close()


class DemoIMU:
    """IMU simulada (movimientos suaves + ruido)."""
    conectado = True

    def __init__(self):
        self.t0 = time.perf_counter()
        self.rng = np.random.default_rng(1)

    def latest(self):
        t = time.perf_counter() - self.t0
        n = self.rng.normal(0, 0.15, 3)
        return ({"R": 3 + 24 * math.sin(2 * math.pi * 0.15 * t) + 3 * math.sin(2 * math.pi * 0.9 * t) + n[0],
                 "P": -5 + 18 * math.sin(2 * math.pi * 0.10 * t + 1.0) + n[1],
                 "Y": 10 + 12 * math.sin(2 * math.pi * 0.07 * t + 2.0) + 0.05 * t + n[2]}, 0.0)


class DemoDobot:
    """Dobot simulado: sigue a la IMU con zona muerta, ganancia, límite y retardo de primer orden."""
    conectado = True

    def __init__(self, imu):
        self.imu = imu
        self.q = [float(v) for v in HOME_JOINTS]
        self.ref = None
        self.t_prev = time.perf_counter()
        self.rng = np.random.default_rng(2)

    def latest(self):
        ahora = time.perf_counter()
        dt, self.t_prev = ahora - self.t_prev, ahora
        imu, _ = self.imu.latest()
        if self.ref is None:
            self.ref = dict(imu)
        alfa = 1 - math.exp(-dt / 0.18)
        for k in KEYS:
            d = zona_muerta(envolver(imu[k] - self.ref[k]), ZONA_MUERTA)
            obj = HOME_JOINTS[DESTINO[k]] + limitar(ganancia_total(k) * d, -LIMITE_DEG, LIMITE_DEG)
            i = DESTINO[k]
            self.q[i] += alfa * (obj - self.q[i])
        return ([v + self.rng.normal(0, 0.03) for v in self.q], 0.0)


# ----------------------------------------------------------------------
# Puente IMU -> Dobot (misma lógica que Comunicacion_STM_CR3.py, modo JOINT)
# ----------------------------------------------------------------------
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

    def dashboard(self, cmd):
        return self._tx(self.dash, cmd)

    def modo_robot(self):
        try:
            m = re.search(r"\{([^}]*)\}", self.dashboard("RobotMode()"))
            return int(float(m.group(1).split(",")[0]))
        except Exception:
            return -1

    def ir_a_home(self, joints, vel):
        j = ",".join(f"{v:.3f}" for v in joints)
        self.move.settimeout(2.0)
        r = self._tx(self.move, f"JointMovJ({j},SpeedJ={vel},AccJ={vel})")
        if r and not r.startswith("0"):
            self.move.settimeout(0.05)
            return False
        self.move.settimeout(120.0)
        r = self._tx(self.move, "Sync()")
        self.move.settimeout(0.05)
        return bool(r) and r.startswith("0")

    def servoj(self, valores):
        r = self._tx(self.move, "ServoJ(" + ",".join(f"{v:.3f}" for v in valores) + ")")
        return not (r and not r.startswith("0"))

    def cerrar(self):
        for s in (self.dash, self.move):
            try:
                s.close()
            except OSError:
                pass


class Puente(threading.Thread):
    """Hilo de control: HOME, enganchar/soltar y ServoJ a SEND_HZ."""

    def __init__(self, imu, log):
        super().__init__(daemon=True)
        self.imu, self.log = imu, log
        self.activo, self.listo, self.enganchado = True, False, False
        self.req_home, self.req_soltar, self.req_enganche = False, False, None
        self.estado = "Conectando…"

    def run(self):
        try:
            robot = Robot(ROBOT_IP)
            robot.dashboard("ClearError()")
            robot.dashboard("EnableRobot()")
            time.sleep(0.5)
            robot.dashboard(f"SpeedFactor({SPEED_FACTOR})")
        except Exception as e:
            self.estado = "Sin conexión"
            self.log(f"Puente: no se pudo conectar con el robot ({e}).")
            return
        self.listo, self.estado = True, "Listo"
        self.log("Robot habilitado. Usa 'Ir a HOME' y luego 'Enganchar'.")
        ref_imu = ref_q = cmd = None
        t_rx_ok = time.time()
        t_tx = t_modo = 0.0
        try:
            while self.activo:
                if self.req_home:
                    self.req_home, self.enganchado = False, False
                    self.estado = "Yendo a HOME…"
                    self.log("Moviendo a HOME…")
                    ok = robot.ir_a_home(HOME_JOINTS, VEL_HOME)
                    self.estado = "En HOME" if ok else "Error en HOME"
                    self.log("HOME alcanzado." if ok else "No se pudo completar HOME (revisa alarma / ClearError).")
                if self.req_soltar:
                    self.req_soltar, self.enganchado, self.estado = False, False, "Soltado"
                    self.log("Soltado: el robot se queda donde está.")
                if self.req_enganche:
                    ref_imu, ref_q = self.req_enganche
                    self.req_enganche = None
                    cmd, self.enganchado, self.estado = list(ref_q), True, "Enganchado"
                    t_rx_ok = time.time()
                    self.log("Enganchado: el robot sigue a la IMU.")

                if self.enganchado:
                    imu, edad = self.imu.latest()
                    if imu is None or edad > TIMEOUT_IMU:
                        self.enganchado, self.estado = False, "Soltado (sin IMU)"
                        self.log("Sin datos de la IMU: soltado por seguridad.")
                        continue
                    if time.time() - t_modo > 0.5:
                        t_modo = time.time()
                        if robot.modo_robot() == 9:
                            self.enganchado, self.estado = False, "ALARMA"
                            self.log("El robot está en ALARMA: soltado. Revisa y usa ClearError().")
                            continue
                    if time.time() - t_tx >= 1.0 / SEND_HZ:
                        t_tx = time.time()
                        for k, cfg in EJES.items():
                            if not cfg["activo"]:
                                continue
                            d = zona_muerta(envolver(imu[k] - ref_imu[k]), ZONA_MUERTA)
                            desp = limitar(cfg["signo"] * cfg["ganancia"] * d, -LIMITE_DEG, LIMITE_DEG)
                            i = DESTINO[k]
                            cmd[i] += limitar(ref_q[i] + desp - cmd[i], -PASO_MAX_DEG, PASO_MAX_DEG)
                        if not robot.servoj(cmd):
                            self.enganchado, self.estado = False, "Rechazado"
                            self.log("El robot rechazó el comando: soltado.")
                time.sleep(0.002)
        finally:
            robot.cerrar()


# ----------------------------------------------------------------------
# Figura (compartida por la vista en vivo y la imagen final)
# ----------------------------------------------------------------------
class Grafica:
    def __init__(self, fig):
        self.fig = fig
        fig.patch.set_facecolor(BG)
        self.ax1 = fig.add_subplot(2, 1, 1)
        self.ax2 = fig.add_subplot(2, 1, 2, sharex=self.ax1)
        fig.subplots_adjust(left=0.075, right=0.985, top=0.80, bottom=0.095, hspace=0.30)
        for ax in (self.ax1, self.ax2):
            ax.set_facecolor(PANEL)
            ax.grid(True, color=GRID, lw=0.7, alpha=0.9)
            ax.tick_params(colors=TXT_DIM, labelsize=9)
            for s in ax.spines.values():
                s.set_color(GRID)
        self.ax1.set_ylabel("Ángulo relativo (°)", color=TXT, fontsize=10)
        self.ax2.set_ylabel("Error (°)", color=TXT, fontsize=10)
        self.ax2.set_xlabel("Tiempo (s)", color=TXT, fontsize=10)
        self.ax1.set_title("Sensor vs. Dobot (°)", color=TXT, fontsize=11, loc="left", pad=34)
        self.ax2.set_title("Margen de error  =  Dobot − (ganancia · Sensor)", color=TXT, fontsize=11, loc="left", pad=6)
        self.ax2.axhline(0, color=TXT_DIM, lw=0.8, alpha=0.7)

        self.ls, self.ld, self.le = {}, {}, {}
        for k in KEYS:
            c = COLOR[k]
            self.ls[k], = self.ax1.plot([], [], color=c, lw=1.9, label=f"Sensor {NOMBRE[k]}")
            self.ld[k], = self.ax1.plot([], [], color=c, lw=1.9, ls="--", label=f"Dobot J{DESTINO[k] + 1} ({NOMBRE[k]})")
            self.le[k], = self.ax2.plot([], [], color=c, lw=1.6, label=f"Error {NOMBRE[k]}")
        self.titulo = fig.text(0.075, 0.965, "Sensor vs. Dobot CR3", color=TXT, fontsize=15, fontweight="bold", va="top")
        self.prop = fig.text(0.075, 0.915, texto_proporcion(), color=ACCENT, fontsize=11, fontweight="bold", va="top")
        self.extra = fig.text(0.985, 0.965, "", color=TXT_DIM, fontsize=9, ha="right", va="top")
        self.visibles = {k: True for k in KEYS}
        self.leyendas()

    def leyendas(self):
        v = [k for k in KEYS if self.visibles[k]]
        for k in KEYS:
            for l in (self.ls[k], self.ld[k], self.le[k]):
                l.set_visible(self.visibles[k])
        h1 = [l for k in v for l in (self.ls[k], self.ld[k])]
        h2 = [self.le[k] for k in v]
        estilo = dict(facecolor=PANEL2, edgecolor=GRID, labelcolor=TXT, fontsize=9, framealpha=0.95)
        if self.ax1.get_legend():
            self.ax1.get_legend().remove()
        if self.ax2.get_legend():
            self.ax2.get_legend().remove()
        if h1:
            self.ax1.legend(handles=h1, loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=3, **estilo)
            self.ax2.legend(handles=h2, loc="upper right", ncol=3, **estilo)

    def datos(self, t, S, D, E):
        for k in KEYS:
            self.ls[k].set_data(t, S[k])
            self.ld[k].set_data(t, D[k])
            self.le[k].set_data(t, E[k])

    def escalar_y(self, S, D, E, margen=0.12, minimo=5.0):
        for ax, series in ((self.ax1, (S, D)), (self.ax2, (E,))):
            vals = [np.asarray(s[k], float) for s in series for k in KEYS if self.visibles[k]]
            vals = [v[np.isfinite(v)] for v in vals]
            vals = np.concatenate([v for v in vals if v.size]) if any(v.size for v in vals) else np.array([0.0])
            lo, hi = float(vals.min()), float(vals.max())
            c, m = (lo + hi) / 2, max((hi - lo) / 2 * (1 + margen), minimo)
            ax.set_ylim(c - m, c + m)


# ----------------------------------------------------------------------
# Aplicación
# ----------------------------------------------------------------------
class App(ctk.CTk):
    def __init__(self, args):
        super().__init__()
        ctk.set_appearance_mode("dark")
        self.args = args
        self.demo, self.controla = args.demo, not (args.demo or args.monitor)
        self.title("Sensor ↔ Dobot CR3 · Monitor en tiempo real")
        self.geometry("1500x860")
        self.minsize(1180, 700)
        self.configure(fg_color=BG)

        self.cola_log = queue.Queue()
        self.hilos = []
        if self.demo:
            self.imu = DemoIMU()
            self.dobot = DemoDobot(self.imu)
        else:
            self.imu = LectorIMU(args.puerto, BAUDIOS, self.log)
            self.dobot = LectorDobot(args.ip, self.log)
            self.hilos += [self.imu, self.dobot]
        self.puente = Puente(self.imu, self.log) if self.controla else None
        if self.puente:
            self.hilos.append(self.puente)

        # Buffers de datos
        self.t = []
        self.S = {k: [] for k in KEYS}
        self.D = {k: [] for k in KEYS}
        self.E = {k: [] for k in KEYS}
        self.extra = []      # (hora, imu abs, q1..q6, enganchado)
        self.t0 = self.ref_imu = self.ref_q = None
        self.suma2 = {k: 0.0 for k in KEYS}
        self.nerr = {k: 0 for k in KEYS}
        self.emax = {k: 0.0 for k in KEYS}
        self.ticks = 0
        self.cerrando = self.guardado = False
        self.t_fin = None

        self.construir_ui()
        self.protocol("WM_DELETE_WINDOW", self.finalizar)
        for h in self.hilos:
            h.start()
        self.log("Modo DEMO (datos simulados)." if self.demo else
                 ("Modo CONTROL: la app mueve el robot (puente IMU→CR3)." if self.controla else
                  "Modo MONITOR: solo lectura, no se envían comandos al robot."))
        self.after(200, self.tick)
        self.after(500, self.refrescar_panel)

    # ---------- UI ----------
    def tarjeta(self, parent, titulo):
        f = ctk.CTkFrame(parent, fg_color=PANEL, corner_radius=14)
        f.pack(fill="x", padx=14, pady=(0, 10))
        ctk.CTkLabel(f, text=titulo.upper(), text_color=TXT_DIM, font=ctk.CTkFont(size=11, weight="bold")
                     ).pack(anchor="w", padx=14, pady=(10, 2))
        return f

    def construir_ui(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        side = ctk.CTkScrollableFrame(self, width=330, fg_color=BG, corner_radius=0)
        side.grid(row=0, column=0, sticky="ns")

        ctk.CTkLabel(side, text="Sensor ↔ Dobot CR3", text_color=TXT, font=ctk.CTkFont(size=22, weight="bold")
                     ).pack(anchor="w", padx=16, pady=(14, 0))
        ctk.CTkLabel(side, text="Monitor de seguimiento en tiempo real", text_color=TXT_DIM
                     ).pack(anchor="w", padx=16, pady=(0, 12))

        # Estado
        c = self.tarjeta(side, "Estado")
        self.lbl_imu = ctk.CTkLabel(c, text="", anchor="w", text_color=TXT)
        self.lbl_dob = ctk.CTkLabel(c, text="", anchor="w", text_color=TXT)
        self.lbl_ctl = ctk.CTkLabel(c, text="", anchor="w", text_color=TXT)
        for l in (self.lbl_imu, self.lbl_dob, self.lbl_ctl):
            l.pack(fill="x", padx=14, pady=1)
        ctk.CTkLabel(c, text="").pack(pady=2)

        # Proporción
        c = self.tarjeta(side, "Proporción sensor : Dobot")
        gs = {round(ganancia_total(k), 6) for k in KEYS}
        grande = f"1 : {fmt_ganancia(ganancia_total('R'))}" if len(gs) == 1 else "por eje"
        ctk.CTkLabel(c, text=grande, text_color=ACCENT, font=ctk.CTkFont(size=34, weight="bold")
                     ).pack(anchor="w", padx=14)
        for k in KEYS:
            ctk.CTkLabel(c, text=f"{NOMBRE[k]} → J{DESTINO[k] + 1}:  Δ Dobot = {fmt_ganancia(ganancia_total(k))} · Δ Sensor",
                         text_color=TXT_DIM, anchor="w").pack(fill="x", padx=14)
        self.lbl_medida = ctk.CTkLabel(c, text="Proporción medida:  —", text_color=TXT, anchor="w",
                                       font=ctk.CTkFont(weight="bold"))
        self.lbl_medida.pack(fill="x", padx=14, pady=(6, 10))

        # Error
        c = self.tarjeta(side, "Margen de error (°)")
        self.lbl_err = {}
        cab = ctk.CTkLabel(c, text=f"{'Eje':<7}{'Actual':>9}{'RMSE':>9}{'Máx':>9}", text_color=TXT_DIM,
                           font=ctk.CTkFont(family="Consolas", size=12), anchor="w")
        cab.pack(fill="x", padx=14)
        for k in KEYS:
            l = ctk.CTkLabel(c, text="", text_color=COLOR[k], anchor="w",
                             font=ctk.CTkFont(family="Consolas", size=13, weight="bold"))
            l.pack(fill="x", padx=14)
            self.lbl_err[k] = l
        self.lbl_info = ctk.CTkLabel(c, text="", text_color=TXT_DIM, anchor="w")
        self.lbl_info.pack(fill="x", padx=14, pady=(4, 10))

        # Ejes visibles
        c = self.tarjeta(side, "Ejes visibles")
        self.sw = {}
        for k in KEYS:
            s = ctk.CTkSwitch(c, text=f"{NOMBRE[k]}  (J{DESTINO[k] + 1})", progress_color=COLOR[k],
                              command=self.cambiar_ejes, text_color=TXT)
            s.select()
            s.pack(anchor="w", padx=14, pady=3)
            self.sw[k] = s
        ctk.CTkLabel(c, text="").pack(pady=1)

        # Acciones
        c = self.tarjeta(side, "Acciones")
        if self.controla:
            ctk.CTkButton(c, text="Ir a HOME", command=self.pedir_home, fg_color=PANEL2, hover_color=GRID
                          ).pack(fill="x", padx=14, pady=3)
            ctk.CTkButton(c, text="Enganchar (robot sigue al sensor)", command=self.enganchar,
                          fg_color=ACCENT).pack(fill="x", padx=14, pady=3)
            ctk.CTkButton(c, text="Soltar", command=self.soltar, fg_color=PANEL2, hover_color=GRID
                          ).pack(fill="x", padx=14, pady=3)
        ctk.CTkButton(c, text="Poner a cero (nueva referencia)", command=self.poner_cero, fg_color=PANEL2,
                      hover_color=GRID).pack(fill="x", padx=14, pady=3)
        ctk.CTkButton(c, text="Finalizar y guardar", command=self.finalizar, fg_color=BAD, hover_color="#d94459",
                      font=ctk.CTkFont(weight="bold")).pack(fill="x", padx=14, pady=(8, 12))

        # Registro
        c = self.tarjeta(side, "Registro")
        self.txt = ctk.CTkTextbox(c, height=130, fg_color=PANEL2, text_color=TXT_DIM, wrap="word")
        self.txt.pack(fill="x", padx=12, pady=(2, 12))
        self.txt.configure(state="disabled")

        # Gráfica
        main = ctk.CTkFrame(self, fg_color=BG, corner_radius=0)
        main.grid(row=0, column=1, sticky="nsew", padx=(0, 10), pady=10)
        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.graf = Grafica(self.fig)
        self.canvas = FigureCanvasTkAgg(self.fig, master=main)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ---------- Acciones ----------
    def log(self, msg):
        self.cola_log.put(f"[{datetime.now():%H:%M:%S}] {msg}")

    def pedir_home(self):
        if messagebox.askokcancel("Ir a HOME", f"El robot irá a HOME {HOME_JOINTS} al {VEL_HOME}%.\n"
                                  "¿Área despejada?", parent=self):
            self.puente.req_home = True

    def enganchar(self):
        imu, ei = self.imu.latest()
        q, eq = self.dobot.latest()
        if imu is None or q is None or ei > 1 or eq > 1:
            self.log("No se puede enganchar: faltan datos de la IMU o del Dobot.")
            return
        self.ref_imu, self.ref_q = dict(imu), list(q)
        self.puente.req_enganche = (dict(imu), list(q))

    def soltar(self):
        self.puente.req_soltar = True

    def poner_cero(self):
        imu, ei = self.imu.latest()
        q, eq = self.dobot.latest()
        if imu is not None and ei < 1:
            self.ref_imu = dict(imu)
        if q is not None and eq < 1:
            self.ref_q = list(q)
        self.log("Referencia (cero) actualizada.")

    def cambiar_ejes(self):
        for k in KEYS:
            self.graf.visibles[k] = bool(self.sw[k].get())
        self.graf.leyendas()
        self.canvas.draw_idle()

    # ---------- Muestreo ----------
    def tick(self):
        if self.cerrando:
            return
        ahora = time.perf_counter()
        try:
            self.muestrear(ahora)
        except Exception:
            traceback.print_exc()
        self.after(int(1000 / GUI_HZ), self.tick)

    def muestrear(self, ahora):
        imu, ei = self.imu.latest()
        q, eq = self.dobot.latest()
        imu_ok, q_ok = imu is not None and ei < 1.0, q is not None and eq < 1.0
        if imu_ok and self.ref_imu is None:
            self.ref_imu = dict(imu)
        if q_ok and self.ref_q is None:
            self.ref_q = list(q)
        if not (imu_ok or q_ok):
            return
        if self.t0 is None:
            self.t0 = ahora
        self.t.append(ahora - self.t0)
        nan = float("nan")
        for k in KEYS:
            s = envolver(imu[k] - self.ref_imu[k]) if imu_ok else nan
            d = envolver(q[DESTINO[k]] - self.ref_q[DESTINO[k]]) if q_ok else nan
            e = d - ganancia_total(k) * s
            self.S[k].append(s)
            self.D[k].append(d)
            self.E[k].append(e)
            if math.isfinite(e):
                self.suma2[k] += e * e
                self.nerr[k] += 1
                self.emax[k] = max(self.emax[k], abs(e))
        self.extra.append((datetime.now(), [imu[k] if imu_ok else nan for k in KEYS],
                           list(q) if q_ok else [nan] * 6,
                           int(bool(self.puente and self.puente.enganchado))))
        self.ticks += 1
        if self.ticks % PLOT_CADA == 0:
            self.redibujar()

    def ventana(self):
        tn = self.t[-1]
        i0 = bisect_left(self.t, tn - VENTANA_S)
        return i0, tn

    def redibujar(self):
        if len(self.t) < 2:
            return
        i0, tn = self.ventana()
        t = np.array(self.t[i0:])
        S = {k: np.array(self.S[k][i0:]) for k in KEYS}
        D = {k: np.array(self.D[k][i0:]) for k in KEYS}
        E = {k: np.array(self.E[k][i0:]) for k in KEYS}
        self.graf.datos(t, S, D, E)
        self.graf.ax1.set_xlim((0, VENTANA_S) if tn < VENTANA_S else (tn - VENTANA_S, tn))
        self.graf.escalar_y(S, D, E)
        self.graf.extra.set_text(f"t = {tn:6.1f} s   ·   {len(self.t)} muestras")
        self.canvas.draw_idle()

    def proporcion_medida(self):
        if len(self.t) < 10:
            return None
        i0 = bisect_left(self.t, self.t[-1] - 10.0)
        num = den = 0.0
        for k in KEYS:
            if not self.graf.visibles[k]:
                continue
            s, d = np.array(self.S[k][i0:]), np.array(self.D[k][i0:])
            m = np.isfinite(s) & np.isfinite(d)
            num += float(np.sum(s[m] * d[m]))
            den += float(np.sum(s[m] ** 2))
        return num / den if den > 25.0 else None

    def refrescar_panel(self):
        if self.cerrando:
            return
        def chip(ok, txt):
            return f"●  {txt}"
        imu_ok = self.imu.conectado
        dob_ok = self.dobot.conectado
        self.lbl_imu.configure(text=chip(imu_ok, "Sensor (STM32): " + ("conectado" if imu_ok else "sin señal")),
                               text_color=OK if imu_ok else BAD)
        self.lbl_dob.configure(text=chip(dob_ok, "Dobot CR3: " + ("conectado" if dob_ok else "sin señal")),
                               text_color=OK if dob_ok else BAD)
        if self.puente:
            en = self.puente.enganchado
            self.lbl_ctl.configure(text=f"●  Control: {self.puente.estado}", text_color=OK if en else WARN)
        else:
            self.lbl_ctl.configure(text="●  " + ("Modo demo" if self.demo else "Modo monitor (solo lectura)"),
                                   text_color=WARN)
        m = self.proporcion_medida()
        self.lbl_medida.configure(text="Proporción medida:  " + ("—" if m is None else f"1 : {m:.3f}"))
        for k in KEYS:
            n = self.nerr[k]
            act = self.E[k][-1] if self.E[k] else float("nan")
            rmse = math.sqrt(self.suma2[k] / n) if n else float("nan")
            self.lbl_err[k].configure(text=f"{NOMBRE[k]:<7}{act:>+9.2f}{rmse:>9.2f}{self.emax[k]:>9.2f}")
        if self.t:
            self.lbl_info.configure(text=f"Duración {self.t[-1]:.1f} s · {len(self.t)} muestras")
        while True:
            try:
                msg = self.cola_log.get_nowait()
            except queue.Empty:
                break
            self.txt.configure(state="normal")
            self.txt.insert("end", msg + "\n")
            self.txt.see("end")
            self.txt.configure(state="disabled")
        self.after(300, self.refrescar_panel)

    # ---------- Exportación ----------
    def guardar(self):
        """Guarda la imagen y el CSV con la fecha/hora exacta de finalización. Devuelve (png, csv) o None."""
        if not self.t:
            return None
        fin = datetime.now()
        marca = fin.strftime("%Y-%m-%d_%H-%M-%S")
        carpeta = carpeta_salida()
        carpeta.mkdir(parents=True, exist_ok=True)
        png = carpeta / f"image_{marca}.png"
        csv_path = carpeta / f"data_{marca}.csv"

        # Imagen: toda la corrida completa
        fig = Figure(figsize=(16, 9), dpi=150)
        g = Grafica(fig)
        g.visibles = dict(self.graf.visibles)
        g.leyendas()
        t = np.array(self.t)
        S = {k: np.array(self.S[k]) for k in KEYS}
        D = {k: np.array(self.D[k]) for k in KEYS}
        E = {k: np.array(self.E[k]) for k in KEYS}
        g.datos(t, S, D, E)
        g.ax1.set_xlim(0, max(float(t[-1]), 1e-3))
        g.escalar_y(S, D, E)
        rmse = " · ".join(f"{NOMBRE[k]} {math.sqrt(self.suma2[k] / self.nerr[k]):.2f}°" if self.nerr[k] else
                          f"{NOMBRE[k]} —" for k in KEYS)
        g.extra.set_text(f"Finalizado: {fin:%Y-%m-%d %H:%M:%S}  ·  {t[-1]:.1f} s  ·  {len(t)} muestras\nRMSE: {rmse}")
        fig.savefig(png, facecolor=fig.get_facecolor())

        # CSV
        nom = [NOMBRE[k].lower() for k in KEYS]
        cab = (["t_s", "fecha_hora"] + [f"sensor_{n}_abs_deg" for n in nom] + [f"sensor_{n}_rel_deg" for n in nom]
               + [f"dobot_J{i}_abs_deg" for i in range(1, 7)]
               + [f"dobot_{n}_J{DESTINO[k] + 1}_rel_deg" for n, k in zip(nom, KEYS)]
               + [f"dobot_esperado_{n}_deg" for n in nom] + [f"error_{n}_deg" for n in nom] + ["enganchado"])
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(cab)
            for i, (hora, imu_abs, q, eng) in enumerate(self.extra):
                s = [self.S[k][i] for k in KEYS]
                d = [self.D[k][i] for k in KEYS]
                esp = [ganancia_total(k) * self.S[k][i] for k in KEYS]
                e = [self.E[k][i] for k in KEYS]
                w.writerow([f"{self.t[i]:.4f}", hora.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]]
                           + [f"{v:.4f}" for v in imu_abs + s + q + d + esp + e] + [eng])
        return png, csv_path

    def finalizar(self):
        if self.cerrando:
            return
        self.cerrando = True
        for h in self.hilos:
            h.activo = False
        rutas = None
        try:
            rutas = self.guardar()
            self.guardado = True
        except Exception:
            traceback.print_exc()
            messagebox.showerror("Error al guardar", traceback.format_exc(), parent=self)
        if rutas:
            print(f"Imagen: {rutas[0]}\nDatos : {rutas[1]}")
            messagebox.showinfo("Datos guardados", f"Imagen:\n{rutas[0]}\n\nDatos:\n{rutas[1]}", parent=self)
        elif not self.guardado or not self.t:
            print("No hubo datos que guardar.")
        self.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="datos simulados, sin hardware")
    ap.add_argument("--monitor", action="store_true", help="solo lectura: no envía comandos al robot")
    ap.add_argument("--puerto", default=PUERTO_STM, help=f"puerto serie de la IMU (def. {PUERTO_STM})")
    ap.add_argument("--ip", default=ROBOT_IP, help=f"IP del Dobot (def. {ROBOT_IP})")
    args = ap.parse_args()
    app = App(args)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if not app.cerrando:
            for h in app.hilos:
                h.activo = False
            try:
                rutas = app.guardar()
                if rutas:
                    print(f"Imagen: {rutas[0]}\nDatos : {rutas[1]}")
            except Exception:
                traceback.print_exc()


if __name__ == "__main__":
    main()