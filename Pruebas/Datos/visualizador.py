#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualizador en tiempo real  ·  Sensor (IMU/STM32)  vs  Dobot CR3  ·  margen de error.

NO hay parametros que editar aqui: todo se lee de TUS dos archivos
    * Comunicacion_STM_CR3.py        (IP, puerto, baudios, HOME, ejes, ganancias, zona muerta, limites...)
    * Orientacion_BIM_ENC_STM.ino    (baudios, campos de la trama, frecuencia, comandos, filtros...)
Con el visualizador abierto, los parametros de ajuste (ganancias, zona muerta, limites...) se recargan
solos cuando guardas el .py.

Novedades de esta version
  * Una fila de graficas por eje (Roll / Pitch / Yaw): izquierda Sensor vs Dobot, derecha margen de error.
  * El eje X es ACUMULATIVO: empieza en 0 cuando se engancha y crece (nunca se desliza). Solo se registra
    mientras el control esta enganchado (en --demo / --monitor empieza con el primer dato).
  * Tabla de telemetria en vivo (sensor, Dobot, esperado, error, RMSE, MAE, maximo) por eje.
  * Registro de mensajes dentro de la barra lateral.

Uso
  python visualizador_tiempo_real.py             # puente + graficas (mueve el robot, como tu script)
  python visualizador_tiempo_real.py --monitor   # solo lectura: no ejecuta el puente ni envia comandos
  python visualizador_tiempo_real.py --demo      # datos simulados con TUS parametros, sin hardware
  opciones: --puente RUTA.py  --ino RUTA.ino  --puerto COMx  --ip X.X.X.X

Dependencias:  pip install customtkinter matplotlib numpy pyserial
"""
import argparse
import ast
import csv
import importlib.util
import math
import queue
import re
import socket
import struct
import threading
import time
import traceback
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
import customtkinter as ctk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ============== AJUSTES DEL VISUALIZADOR (nada del robot ni del firmware) ==============
NOMBRE_PUENTE = "Comunicacion_STM_CR3.py"
RUTA_PUENTE   = None      # None = autodetectar dentro del proyecto (carpeta Firmware)
RUTA_INO      = None      # None = autodetectar (*.ino que contiene Serial.begin)
RUTA_SALIDA   = None      # None = <proyecto>/Pruebas/Datos
X_MIN_S       = 10.0      # ancho inicial del eje X (s)
X_PASO_S      = 10.0      # el eje X crece en saltos de N segundos (menos "temblor" visual)
MAX_PUNTOS    = 3000      # maximo de puntos dibujados por linea (decimacion solo visual)
GUI_HZ        = 30        # muestras guardadas por segundo
PLOT_CADA     = 3         # redibuja cada N muestras
REVISAR_CFG_S = 2.0       # cada cuantos segundos mira si cambiaron tus archivos
# =======================================================================================

KEYS   = ("R", "P", "Y")
NOMBRE = {"R": "Roll", "P": "Pitch", "Y": "Yaw"}
COLOR  = {"R": "#e5484d", "P": "#0f9d8a", "Y": "#e08a00"}

# Paleta (tema claro)
BG, CARD, CARD2, BORDER = "#f2f5fb", "#ffffff", "#f6f8fd", "#e2e8f3"
TXT, DIM, GRIDC = "#1a2238", "#68748f", "#e6ebf4"
ACCENT, ACCENT_BG = "#3a63f0", "#e8eeff"
OK, OK_BG, BAD, BAD_BG, WARN, WARN_BG = "#14935f", "#e3f6ee", "#d93a49", "#fde9eb", "#c27a00", "#fff3dc"

# Protocolo de realimentacion del Dobot CR (puerto 30004)
PUERTO_FB, TAM_PAQUETE, TEST_VALUE = 30004, 1440, 0x0123456789ABCDEF
OFF_MODO, OFF_TEST, OFF_Q, OFF_POSE = 24, 48, 432, 624
POSE_NOMBRES = ("X", "Y", "Z", "Rx", "Ry", "Rz")

DEFAULTS_PY = dict(
    ROBOT_IP="172.16.2.62", PUERTO_STM="COM14", BAUDIOS=115200, SIMULAR=False, MODO="JOINT",
    SPEED_FACTOR=30, SEND_HZ=30, IR_A_HOME=True, HOME_JOINTS=(90, 0, 0, 0, -90, 0), VEL_HOME=30,
    CONFIRMAR_HOME=True, AUTO_ENGANCHAR=False, TOLERANCIA_HOME=1.0,
    DESTINO={"JOINT": {"R": 5, "P": 3, "Y": 4}, "POSE": {"R": 3, "P": 3, "Y": 4}},
    EJES={k: {"activo": True, "ganancia": 1.0, "signo": 1.0} for k in KEYS},
    ZONA_MUERTA=2.0, LIMITE_DEG=30.0, PASO_MAX_DEG=3.0, TIMEOUT_IMU=0.3)
# Se reaplican en caliente (el puente los lee en cada ciclo); el resto requiere reiniciar
AJUSTABLES = ("ZONA_MUERTA", "LIMITE_DEG", "PASO_MAX_DEG", "SEND_HZ", "TIMEOUT_IMU")
NAN = float("nan")


# ======================================================================
# Matematica
# ======================================================================
def envolver(a):
    return (a + 180.0) % 360.0 - 180.0


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


def zona_muerta(d, z):
    return 0.0 if abs(d) <= z else d - math.copysign(z, d)


def fmt(v, nd=3):
    return f"{v:.{nd}g}"


def num(v, nd=2, signo=False):
    """Formato de celda: guion si no es finito."""
    if v is None or not math.isfinite(v):
        return "—"
    return f"{v:+.{nd}f}" if signo else f"{v:.{nd}f}"


# ======================================================================
# Lectura de TUS archivos
# ======================================================================
def leer_py(path):
    """Constantes MAYUSCULAS del puente, leidas con ast (sin ejecutar nada)."""
    out = {}
    tree = ast.parse(Path(path).read_text(encoding="utf-8", errors="ignore"))
    for n in tree.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
            nom = n.targets[0].id
            if nom.isupper():
                try:
                    out[nom] = ast.literal_eval(n.value)
                except Exception:
                    pass
    return out


def leer_ino(path):
    """Datos utiles del firmware, extraidos con expresiones regulares."""
    s = Path(path).read_text(encoding="utf-8", errors="ignore")

    def f(pat, cast=float):
        m = re.search(pat, s)
        return cast(m.group(1)) if m else None
    d = {"baud": f(r"Serial\.begin\(\s*(\d+)", int),
         "campos": re.findall(r'Serial\.print(?:ln)?\(\s*"(?:\\t)?([A-Za-z]\w*):"\s*\)', s),
         "periodo_ms": f(r"millis\(\)\s*-\s*\w+\s*>\s*(\d+)", int),
         "comandos": {m.group(1): (m.group(2) or "").strip()
                      for m in re.finditer(r"cmd\s*==\s*'(\w)'\s*\)\s*\{[ \t]*(?://\s*([^\n]*))?", s)},
         "deadband": f(r"gyro_deadband\s*=\s*([\d.]+)"),
         "enc_hz": f(r"ENC_RATE_HZ\s+(\d+)", int),
         "enc_timers": re.findall(r"\{\s*(TIM\d+)\s*,\s*(?:true|false)", s),
         "cal_n": f(r"const\s+int\s+N\s*=\s*(\d+)", int),
         "q_angle": f(r"Q_angle\s*=\s*([\d.]+)"), "r_angle": f(r"R_angle\s*=\s*([\d.]+)")}
    m = re.search(r"initBiquad\(\s*\w+\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\)", s)
    d["biquad"] = (float(m.group(1)), float(m.group(2))) if m else None
    m = re.search(r'"BMI160:\s*([^"]+)"', s)
    d["imu"] = m.group(1) if m else None
    for k, desc in (("z", "Cero de encoders"), ("c", "Recalibrar IMU")):
        if k in d["comandos"] and not d["comandos"][k]:
            d["comandos"][k] = desc
    return d


def buscar_archivos(ruta_py, ruta_ino):
    """Autodetecta ambos archivos dentro del proyecto (carpeta Firmware) o junto al visualizador."""
    py = Path(ruta_py) if ruta_py else None
    ino = Path(ruta_ino) if ruta_ino else None
    aqui = Path(__file__).resolve().parent
    bases = [p for p in [aqui, *aqui.parents, Path.cwd()]]
    firmware = [p / "Firmware" for p in bases if (p / "Firmware").is_dir()]
    if not py:
        for d in firmware:
            py = next(iter(d.rglob(NOMBRE_PUENTE)), None)
            if py:
                break
        py = py or next((p / NOMBRE_PUENTE for p in bases if (p / NOMBRE_PUENTE).exists()), None)
    if not ino:
        for d in firmware + [aqui, Path.cwd()]:
            for c in d.rglob("*.ino") if d.name == "Firmware" else d.glob("*.ino"):
                if "Serial.begin" in c.read_text(encoding="utf-8", errors="ignore"):
                    ino = c
                    break
            if ino:
                break
    if not py or not ino:
        raiz = tk.Tk()
        raiz.withdraw()
        if not py:
            py = filedialog.askopenfilename(title=f"Selecciona {NOMBRE_PUENTE}", filetypes=[("Python", "*.py")])
        if not ino:
            ino = filedialog.askopenfilename(title="Selecciona el .ino del firmware", filetypes=[("Arduino", "*.ino")])
        raiz.destroy()
    if not py or not ino:
        raise SystemExit("No se encontraron el puente .py y/o el firmware .ino")
    return Path(py), Path(ino)


def carpeta_salida():
    if RUTA_SALIDA:
        return Path(RUTA_SALIDA)
    aqui = Path(__file__).resolve().parent
    for p in [aqui, *aqui.parents]:
        if p.name.lower() == "pruebas":
            return p / "Datos"
        if p.name.startswith("7_Dofs_Haptic_Device_STM32"):
            return p / "Pruebas" / "Datos"
    return Path.cwd() / "Pruebas" / "Datos"


class Config:
    """Configuracion efectiva = lo que dicen tus archivos. Se vigila su fecha de modificacion."""

    def __init__(self, ruta_py, ruta_ino, overrides=None):
        self.ruta_py, self.ruta_ino = ruta_py, ruta_ino
        self.overrides = overrides or {}
        self.reinicio_pendiente, self.ajustes_pendientes = [], {}
        self.py = self._leer_py()
        self.ino = leer_ino(ruta_ino)
        self.mt = self._mtimes()

    def _mtimes(self):
        return (self.ruta_py.stat().st_mtime, self.ruta_ino.stat().st_mtime)

    def _leer_py(self):
        d = {**DEFAULTS_PY, **leer_py(self.ruta_py), **self.overrides}
        d["EJES"] = {k: {**DEFAULTS_PY["EJES"][k], **d["EJES"].get(k, {})} for k in KEYS}
        return d

    # --- accesores ---
    @property
    def modo(self):
        return str(self.py["MODO"]).upper()

    def destino(self, k):
        return self.py["DESTINO"][self.modo][k]

    def nombre_destino(self, k):
        i = self.destino(k)
        return f"J{i + 1}" if self.modo == "JOINT" else POSE_NOMBRES[i]

    def activo(self, k):
        return bool(self.py["EJES"][k]["activo"])

    def ganancia(self, k):
        e = self.py["EJES"][k]
        return float(e["signo"]) * float(e["ganancia"])

    def revisar(self):
        """True si cambiaron los archivos. Los ajustes quedan pendientes; el resto pide reiniciar."""
        try:
            mt = self._mtimes()
        except OSError:
            return False
        if mt == self.mt:
            return False
        try:
            self.ino = leer_ino(self.ruta_ino)
            nuevo = self._leer_py()
        except Exception:
            return False          # archivo a medio guardar: se reintenta en el siguiente ciclo
        self.mt = mt
        self.reinicio_pendiente = [k for k, v in nuevo.items()
                                   if k not in AJUSTABLES and k != "EJES" and v != self.py.get(k)]
        if any(nuevo["EJES"][k]["activo"] != self.py["EJES"][k]["activo"] for k in KEYS):
            self.reinicio_pendiente.append("EJES.activo")
        self.ajustes_pendientes = {k: nuevo[k] for k in AJUSTABLES}
        self.ajustes_pendientes["EJES"] = {k: {"ganancia": nuevo["EJES"][k]["ganancia"],
                                               "signo": nuevo["EJES"][k]["signo"]} for k in KEYS}
        return True

    def aplicar_ajustes(self, modulo_puente=None):
        for k, v in self.ajustes_pendientes.items():
            if k == "EJES":
                for ej, g in v.items():
                    self.py["EJES"][ej].update(g)
                    if modulo_puente:
                        modulo_puente.EJES[ej].update(g)
            else:
                self.py[k] = v
                if modulo_puente:
                    setattr(modulo_puente, k, v)
        self.ajustes_pendientes = {}


# ======================================================================
# Fuentes de datos
# ======================================================================
class FuenteIMU:
    """Ultima trama de la IMU. La alimenta el puente (via TeeSerial), LectorIMU o la demo."""

    def __init__(self):
        self.lock = threading.Lock()
        self.dato, self.t_rx = None, 0.0
        self.marcas = deque(maxlen=120)
        self.conectado, self.ser = False, None

    def feed(self, raw):
        linea = raw.decode("utf-8", errors="ignore") if isinstance(raw, (bytes, bytearray)) else raw
        d = dict(re.findall(r"([A-Za-z]\w*):(-?\d+(?:\.\d+)?)", linea))
        if all(k in d for k in KEYS):
            ahora = time.time()
            with self.lock:
                self.dato = {k: float(v) for k, v in d.items()}
                self.t_rx = ahora
                self.marcas.append(ahora)

    def latest(self):
        with self.lock:
            return (dict(self.dato), time.time() - self.t_rx) if self.dato else (None, 1e9)

    def hz(self):
        with self.lock:
            m = list(self.marcas)
        if len(m) < 5 or time.time() - m[-1] > 1.0:
            return 0.0
        return (len(m) - 1) / max(m[-1] - m[0], 1e-6)

    def escribir(self, datos):
        try:
            if self.ser:
                self.ser.write(datos)
                return True
        except Exception:
            pass
        return False


class LectorIMU(threading.Thread):
    """Lee el puerto serie por su cuenta (modo --monitor o si el puente termina)."""

    def __init__(self, fuente, puerto, baud, log):
        super().__init__(daemon=True)
        self.f, self.puerto, self.baud, self.log, self.activo = fuente, puerto, baud, log, True

    def run(self):
        import serial
        avisado = False
        while self.activo:
            try:
                ser = serial.Serial(self.puerto, self.baud, timeout=0.2)
            except Exception as e:
                if not avisado:
                    self.log(f"IMU: no se pudo abrir {self.puerto} ({e}). Reintentando…")
                    avisado = True
                time.sleep(1.0)
                continue
            avisado, self.f.conectado, self.f.ser = False, True, ser
            self.log(f"IMU conectada en {self.puerto}.")
            try:
                while self.activo:
                    linea = ser.readline()
                    if linea:
                        self.f.feed(linea)
            except Exception as e:
                self.log(f"IMU: conexion perdida ({e}).")
            finally:
                self.f.conectado, self.f.ser = False, None
                try:
                    ser.close()
                except Exception:
                    pass


class LectorDobot(threading.Thread):
    """QActual y pose TCP del CR3 por el puerto 30004 (solo lectura)."""

    def __init__(self, ip, log):
        super().__init__(daemon=True)
        self.ip, self.log, self.activo = ip, log, True
        self.lock = threading.Lock()
        self.ultimo, self.t_rx, self.conectado = None, 0.0, False

    def latest(self):
        with self.lock:
            return (dict(self.ultimo), time.time() - self.t_rx) if self.ultimo else (None, 1e9)

    def run(self):
        avisado = False
        while self.activo:
            try:
                sock = socket.create_connection((self.ip, PUERTO_FB), timeout=3)
            except Exception as e:
                self.conectado = False
                if not avisado:
                    self.log(f"Dobot: sin conexion a {self.ip}:{PUERTO_FB} ({e}). Reintentando…")
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
                            self.ultimo = {"q": list(struct.unpack_from("<6d", pkt, OFF_Q)),
                                           "pose": list(struct.unpack_from("<6d", pkt, OFF_POSE)),
                                           "modo": struct.unpack_from("<Q", pkt, OFF_MODO)[0]}
                            self.t_rx = time.time()
            except Exception as e:
                self.log(f"Dobot: conexion perdida ({e}).")
            finally:
                self.conectado = False
                sock.close()


class DemoIMU(FuenteIMU, threading.Thread):
    """IMU simulada: genera lineas con el MISMO formato que tu firmware, a su frecuencia."""

    def __init__(self, cfg):
        FuenteIMU.__init__(self)
        threading.Thread.__init__(self, daemon=True)
        self.cfg, self.activo, self.conectado = cfg, True, True

    def run(self):
        rng, t0 = np.random.default_rng(1), time.perf_counter()
        per = (self.cfg.ino.get("periodo_ms") or 20) / 1000.0
        while self.activo:
            t = time.perf_counter() - t0
            n = rng.normal(0, 0.15, 3)
            R = 3 + 24 * math.sin(2 * math.pi * 0.15 * t) + 3 * math.sin(2 * math.pi * 0.9 * t) + n[0]
            P = -5 + 18 * math.sin(2 * math.pi * 0.10 * t + 1.0) + n[1]
            Y = 10 + 12 * math.sin(2 * math.pi * 0.07 * t + 2.0) + 0.05 * t + n[2]
            e = [int(400 * math.sin(2 * math.pi * f * t + i)) for i, f in enumerate((0.1, 0.13, 0.17, 0.21))]
            self.feed(f"T:{int(t * 1000)}P:{P:.3f}\tR:{R:.3f}\tY:{Y:.3f}"
                      f"\tE1:{e[0]}\tE2:{e[1]}\tE3:{e[2]}\tE4:{e[3]}")
            time.sleep(per)

    def escribir(self, datos):
        return True


class DemoDobot:
    """Dobot simulado: sigue a la IMU con TU zona muerta, ganancia, limite y retardo."""

    def __init__(self, cfg, imu):
        self.cfg, self.imu, self.conectado = cfg, imu, True
        self.q = [float(v) for v in cfg.py["HOME_JOINTS"]]
        self.pose = [0.0] * 6
        self.ref, self.t_prev = None, time.perf_counter()
        self.rng = np.random.default_rng(2)

    def latest(self):
        ahora = time.perf_counter()
        dt, self.t_prev = ahora - self.t_prev, ahora
        imu, _ = self.imu.latest()
        if imu is None:
            return None, 1e9
        if self.ref is None:
            self.ref = dict(imu)
        c, alfa = self.cfg, 1 - math.exp(-dt / 0.18)
        campo = self.q if c.modo == "JOINT" else self.pose
        base = c.py["HOME_JOINTS"] if c.modo == "JOINT" else [0] * 6
        for k in KEYS:
            if not c.activo(k):
                continue
            d = zona_muerta(envolver(imu[k] - self.ref[k]), c.py["ZONA_MUERTA"])
            lim = c.py["LIMITE_DEG"]
            i = c.destino(k)
            campo[i] += alfa * (base[i] + limitar(c.ganancia(k) * d, -lim, lim) - campo[i])
        ruido = lambda v: [x + self.rng.normal(0, 0.03) for x in v]
        return {"q": ruido(self.q), "pose": ruido(self.pose), "modo": 5}, 0.0


# ======================================================================
# Puente: se ejecuta TU main() con la E/S redirigida a la interfaz
# ======================================================================
class PuenteExterno(threading.Thread):
    def __init__(self, ruta, fuente, log, dialogo, overrides):
        super().__init__(daemon=True)
        self.ruta, self.f, self.log, self.dialogo, self.overrides = ruta, fuente, log, dialogo, overrides
        self.cmds, self.detener, self.mod = queue.Queue(), False, None
        self.terminado = False

    @property
    def estado(self):
        return self.mod.estado if self.mod else {}

    def enviar(self, c):
        self.cmds.put(c)

    def _input(self, prompt=""):
        if prompt:                                     # confirmacion de HOME
            if not self.dialogo(prompt):
                raise KeyboardInterrupt
            return ""
        while not self.detener:                        # hilo de teclado del puente
            try:
                return self.cmds.get(timeout=0.2)
            except queue.Empty:
                pass
        raise SystemExit

    def _print(self, *a, **k):
        for linea in " ".join(map(str, a)).splitlines():
            if linea.strip():
                self.log(linea.strip())

    def run(self):
        f = self.f
        try:
            spec = importlib.util.spec_from_file_location("puente_cr3_usuario", self.ruta)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            for k, v in self.overrides.items():
                setattr(mod, k, v)
            real = mod.serial

            class TeeSerial:                           # ve cada linea que lee tu puente
                def __init__(s, *a, **kw):
                    s._s = real.Serial(*a, **kw)
                    f.conectado, f.ser = True, s._s

                def readline(s, *a, **kw):
                    l = s._s.readline(*a, **kw)
                    if l:
                        f.feed(l)
                    return l

                def close(s):
                    f.conectado, f.ser = False, None
                    return s._s.close()

                def __getattr__(s, n):
                    return getattr(s._s, n)

            class Shim:
                Serial = TeeSerial

                def __getattr__(s, n):
                    return getattr(real, n)

            mod.serial, mod.input, mod.print = Shim(), self._input, self._print
            self.mod = mod
            mod.main()
        except SystemExit:
            pass
        except BaseException:
            self.log("El puente fallo:\n" + traceback.format_exc(limit=3))
        finally:
            self.terminado, f.conectado = True, False
            self.log("Puente detenido." if self.detener else "El puente termino.")

    def parar(self):
        self.detener = True
        if self.mod:
            self.mod.estado["salir"] = True


# ======================================================================
# Figuras
# ======================================================================
def estilo_ejes(ax):
    ax.set_facecolor(CARD)
    ax.grid(True, color=GRIDC, lw=0.8)
    ax.tick_params(colors=DIM, labelsize=8.5)
    for s in ax.spines.values():
        s.set_color(BORDER)


def escalar(ax, listas, minimo):
    vals = [v[np.isfinite(v)] for v in listas]
    vals = [v for v in vals if v.size]
    if not vals:
        return
    v = np.concatenate(vals)
    lo, hi = float(v.min()), float(v.max())
    c, m = (lo + hi) / 2, max((hi - lo) / 2 * 1.15, minimo)
    ax.set_ylim(c - m, c + m)


def texto_proporcion(cfg):
    gs = {round(cfg.ganancia(k), 6) for k in KEYS if cfg.activo(k)} or {round(cfg.ganancia("R"), 6)}
    if len(gs) == 1:
        g = gs.pop()
        return f"Proporción Sensor : Dobot = 1 : {fmt(g)}     (Δ Dobot = {fmt(g)} · Δ Sensor)"
    return "Proporción Sensor : Dobot   " + "  |  ".join(
        f"{NOMBRE[k]} 1 : {fmt(cfg.ganancia(k))}" for k in KEYS if cfg.activo(k))


def xmax_acumulativo(tn):
    """El eje X nace en 0 y solo crece, en saltos de X_PASO_S."""
    return max(X_MIN_S, math.ceil((tn + 1e-6) / X_PASO_S) * X_PASO_S)


class GraficaSeguimiento:
    """3 filas (una por eje) x 2 columnas: [Sensor vs Dobot | Margen de error]. Eje X compartido."""

    def __init__(self, fig, cfg):
        self.fig, self.cfg = fig, cfg
        fig.patch.set_facecolor(CARD)
        gs = fig.add_gridspec(3, 2, left=0.07, right=0.985, top=0.865, bottom=0.115,
                              hspace=0.30, wspace=0.13, width_ratios=[1.45, 1])
        self.axs, self.axe = {}, {}
        for i, k in enumerate(KEYS):
            self.axs[k] = fig.add_subplot(gs[i, 0], sharex=self.axs[KEYS[0]] if i else None)
            self.axe[k] = fig.add_subplot(gs[i, 1], sharex=self.axs[KEYS[0]])
            for ax in (self.axs[k], self.axe[k]):
                estilo_ejes(ax)
                if i < 2:
                    ax.tick_params(labelbottom=False)
            self.axs[k].set_ylabel(f"{NOMBRE[k]} (°)", color=COLOR[k], fontsize=10, fontweight="bold")
            self.axe[k].axhline(0, color=DIM, lw=0.9, alpha=0.7)
        self.axs[KEYS[0]].set_title("Sensor vs. Dobot  (° relativos al enganche)", color=TXT, fontsize=10,
                                    loc="left", pad=6)
        self.axe[KEYS[0]].set_title("Margen de error  =  Dobot − esperado(Sensor)", color=TXT, fontsize=10,
                                    loc="left", pad=6)
        for ax in (self.axs[KEYS[-1]], self.axe[KEYS[-1]]):
            ax.set_xlabel("Tiempo desde el enganche (s)", color=TXT, fontsize=9.5)
        self.axs[KEYS[0]].set_xlim(0, X_MIN_S)

        self.ls, self.ld, self.le = {}, {}, {}
        for k in KEYS:
            self.ls[k], = self.axs[k].plot([], [], color=COLOR[k], lw=1.9)
            self.ld[k], = self.axs[k].plot([], [], color=COLOR[k], lw=1.9, ls="--")
            self.le[k], = self.axe[k].plot([], [], color=COLOR[k], lw=1.5)

        fig.text(0.07, 0.985, "Sensor vs. Dobot CR3", color=TXT, fontsize=14, fontweight="bold", va="top")
        self.prop = fig.text(0.07, 0.940, "", color=ACCENT, fontsize=10, fontweight="bold", va="top")
        self.extra = fig.text(0.985, 0.985, "", color=DIM, fontsize=9, ha="right", va="top")
        self.pie = fig.text(0.07, 0.012, "", color=DIM, fontsize=8, va="bottom")
        self.espera = fig.text(0.53, 0.5, "", color=DIM, fontsize=15, ha="center", va="center",
                               bbox=dict(boxstyle="round,pad=0.8", fc=CARD2, ec=BORDER))
        self.visibles = {k: cfg.activo(k) for k in KEYS}
        self.refrescar_texto()

    def refrescar_texto(self):
        c = self.cfg
        for k in KEYS:
            suf = "" if c.activo(k) else " [inactivo]"
            self.ls[k].set_label(f"Sensor {NOMBRE[k]}")
            self.ld[k].set_label(f"Dobot {c.nombre_destino(k)}{suf}")
        self.prop.set_text(texto_proporcion(c))
        self.pie.set_text(f"Puente: velocidad {fmt(c.py['SPEED_FACTOR'])} % · "
                          f"zona muerta {fmt(c.py['ZONA_MUERTA'])}° · límite ±{fmt(c.py['LIMITE_DEG'])}° · "
                          f"paso máx {fmt(c.py['PASO_MAX_DEG'])}°/envío · {fmt(c.py['SEND_HZ'])} Hz · modo {c.modo}")
        self.leyendas()

    def leyendas(self):
        est = dict(facecolor=CARD, edgecolor=BORDER, labelcolor=TXT, fontsize=8, framealpha=0.92,
                   loc="upper right", ncol=2, borderpad=0.4)
        for k in KEYS:
            vis = self.visibles[k]
            self.axs[k].set_visible(vis)
            self.axe[k].set_visible(vis)
            if self.axs[k].get_legend():
                self.axs[k].get_legend().remove()
            if vis:
                self.axs[k].legend(handles=[self.ls[k], self.ld[k]], **est)

    def datos(self, t, S, D, E):
        for k in KEYS:
            self.ls[k].set_data(t, S[k])
            self.ld[k].set_data(t, D[k])
            self.le[k].set_data(t, E[k])

    def escalar_y(self, S, D, E):
        for k in KEYS:
            if self.visibles[k]:
                escalar(self.axs[k], [np.asarray(S[k], float), np.asarray(D[k], float)], 5.0)
                escalar(self.axe[k], [np.asarray(E[k], float)], 2.0)

    def set_xmax(self, xmax):
        self.axs[KEYS[0]].set_xlim(0, xmax)


class GraficaSensor:
    """Pestaña 2: IMU absoluta (R/P/Y) y encoders del gimbal."""

    def __init__(self, fig, enc_keys):
        fig.patch.set_facecolor(CARD)
        self.a1, self.a2 = fig.add_subplot(2, 1, 1), fig.add_subplot(2, 1, 2)
        self.a2.sharex(self.a1)
        fig.subplots_adjust(left=0.075, right=0.985, top=0.90, bottom=0.11, hspace=0.32)
        for ax in (self.a1, self.a2):
            estilo_ejes(ax)
        self.a1.tick_params(labelbottom=False)
        self.a1.set_title("Orientación absoluta de la IMU (°)", color=TXT, fontsize=11, loc="left", pad=8)
        self.a2.set_title("Encoders del gimbal (cuentas)", color=TXT, fontsize=11, loc="left", pad=8)
        self.a1.set_ylabel("Ángulo (°)", color=TXT)
        self.a2.set_ylabel("Cuentas", color=TXT)
        self.a2.set_xlabel("Tiempo desde el enganche (s)", color=TXT)
        self.a1.set_xlim(0, X_MIN_S)
        self.l1 = {k: self.a1.plot([], [], color=COLOR[k], lw=1.8, label=NOMBRE[k])[0] for k in KEYS}
        pal = ["#3a63f0", "#8a4fe0", "#0b9fcf", "#e0508a", "#7a8a2a", "#444444"]
        self.l2 = {e: self.a2.plot([], [], color=pal[i % len(pal)], lw=1.6, label=e)[0] for i, e in enumerate(enc_keys)}
        est = dict(facecolor=CARD, edgecolor=BORDER, labelcolor=TXT, fontsize=9, ncol=4,
                   loc="lower right", bbox_to_anchor=(1.0, 1.0))
        self.a1.legend(**est)
        if enc_keys:
            self.a2.legend(**est)


# ======================================================================
# Aplicacion
# ======================================================================
class App(ctk.CTk):
    def __init__(self, args, cfg):
        super().__init__()
        ctk.set_appearance_mode("light")
        self.args, self.cfg = args, cfg
        self.demo = args.demo
        self.controla = not (args.demo or args.monitor)
        self.title("Sensor ↔ Dobot CR3 · Monitor en tiempo real")
        self.geometry("1600x920")
        self.minsize(1280, 760)
        self.configure(fg_color=BG)

        self.cola_log, self.cola_dialogos = queue.Queue(), queue.Queue()
        self.hilos, self.lector_imu, self._iniciada = [], None, False
        self.imu = DemoIMU(cfg) if self.demo else FuenteIMU()
        if self.demo:
            self.dobot = DemoDobot(cfg, self.imu)
            self.hilos.append(self.imu)
        else:
            self.dobot = LectorDobot(cfg.py["ROBOT_IP"], self.log)
            self.hilos.append(self.dobot)
        self.puente = None
        if self.controla:
            ov = {k: v for k, v in (("PUERTO_STM", args.puerto), ("ROBOT_IP", args.ip)) if v}
            self.puente = PuenteExterno(cfg.ruta_py, self.imu, self.log, self.pedir_dialogo, ov)
            self.hilos.append(self.puente)
        elif not self.demo:
            self.iniciar_lector_imu()

        self.enc_keys = [e for e in cfg.ino.get("campos", []) if re.fullmatch(r"E\d+", e)] or ["E1", "E2", "E3", "E4"]
        self.vivo = {k: dict(sa=NAN, sr=NAN, dr=NAN, x=NAN, e=NAN) for k in KEYS}
        self.ref_imu = self.ref_fb = None
        self.ticks, self.grab_prev, self.grabando = 0, False, False
        self.t_last = None
        self.modelo = "prop"
        self.cerrando = self.guardado = False
        self.t_cfg = time.time()
        self.reiniciar_datos()

        self.construir_ui()
        self.protocol("WM_DELETE_WINDOW", self.finalizar)
        for h in self.hilos:
            h.start()
        self.log("DEMO: datos simulados con tus parámetros." if self.demo else
                 ("CONTROL: se ejecuta tu Comunicacion_STM_CR3.py (mueve el robot)." if self.controla else
                  "MONITOR: solo lectura, no se envían comandos."))
        if self.controla:
            self.log("El registro y el eje de tiempo empiezan al ENGANCHAR.")
        self.after(200, self.tick)
        self.after(400, self.refrescar_panel)
        self.after(100, self.poll_dialogos)

    # ---------- datos acumulados ----------
    def reiniciar_datos(self):
        self.t = []
        self.S, self.D, self.E, self.X, self.A = ({k: [] for k in KEYS} for _ in range(5))
        self.ENC = {e: [] for e in self.enc_keys}
        self.extra = []
        self.t_acum, self.t_last = 0.0, None
        self.suma2 = {k: 0.0 for k in KEYS}
        self.sumaabs = {k: 0.0 for k in KEYS}
        self.nerr = {k: 0 for k in KEYS}
        self.emax = {k: 0.0 for k in KEYS}

    def reiniciar_registro(self):
        self.reiniciar_datos()
        if hasattr(self, "graf"):
            self.graf.datos([], {k: [] for k in KEYS}, {k: [] for k in KEYS}, {k: [] for k in KEYS})
            self.graf.set_xmax(X_MIN_S)
            self.canvas.draw_idle()
            self.canvas2.draw_idle()
        self.log("Registro reiniciado: el eje de tiempo vuelve a 0.")

    # ---------- utilidades ----------
    def log(self, msg):
        self.cola_log.put(f"[{datetime.now():%H:%M:%S}] {msg}")

    def iniciar_lector_imu(self):
        puerto = self.args.puerto or self.cfg.py["PUERTO_STM"]
        self.lector_imu = LectorIMU(self.imu, puerto, self.cfg.py["BAUDIOS"], self.log)
        self.hilos.append(self.lector_imu)
        if self._iniciada:
            self.lector_imu.start()

    def pedir_dialogo(self, prompt):
        """Llamado desde el hilo del puente: bloquea hasta que el usuario responde."""
        ev, res = threading.Event(), [False]
        self.cola_dialogos.put((prompt, ev, res))
        while not ev.wait(0.2):
            if self.cerrando:
                return False
        return res[0]

    def poll_dialogos(self):
        try:
            prompt, ev, res = self.cola_dialogos.get_nowait()
            txt = re.sub(r"Enter para continuar.*$", "", prompt.strip(), flags=re.S).strip()
            res[0] = messagebox.askokcancel("Confirmación del puente", txt + "\n\n¿Continuar?", parent=self)
            ev.set()
        except queue.Empty:
            pass
        if not self.cerrando:
            self.after(100, self.poll_dialogos)

    def esta_enganchado(self):
        return bool(self.puente and self.puente.estado.get("enganchado"))

    # ---------- construcción de la UI ----------
    def card(self, parent, titulo, pad=(0, 10)):
        f = ctk.CTkFrame(parent, fg_color=CARD, corner_radius=14, border_width=1, border_color=BORDER)
        f.pack(fill="x", padx=10, pady=pad)
        if titulo:
            ctk.CTkLabel(f, text=titulo.upper(), text_color=DIM, font=ctk.CTkFont(size=11, weight="bold")
                         ).pack(anchor="w", padx=14, pady=(10, 4))
        return f

    def chip(self, parent):
        l = ctk.CTkLabel(parent, text="", corner_radius=14, height=30, font=ctk.CTkFont(size=12, weight="bold"))
        l.pack(side="left", padx=4)
        return l

    def set_chip(self, l, texto, tono):
        fg, bg = {"ok": (OK, OK_BG), "bad": (BAD, BAD_BG), "warn": (WARN, WARN_BG), "info": (ACCENT, ACCENT_BG)}[tono]
        l.configure(text=f"   {texto}   ", text_color=fg, fg_color=bg)

    def construir_ui(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # ---- Encabezado ----
        hd = ctk.CTkFrame(self, fg_color=CARD, corner_radius=0, border_width=0, height=64)
        hd.grid(row=0, column=0, columnspan=2, sticky="ew")
        hd.grid_propagate(False)
        ctk.CTkLabel(hd, text="Sensor ↔ Dobot CR3", text_color=TXT, font=ctk.CTkFont(size=22, weight="bold")
                     ).pack(side="left", padx=(20, 4), pady=10)
        ctk.CTkLabel(hd, text="Seguimiento háptico en tiempo real", text_color=DIM).pack(side="left", padx=6)
        ch = ctk.CTkFrame(hd, fg_color="transparent")
        ch.pack(side="right", padx=14)
        self.ch_imu, self.ch_dob, self.ch_ctl, self.ch_modo, self.ch_t = (self.chip(ch) for _ in range(5))

        # ---- Barra lateral ----
        side = ctk.CTkScrollableFrame(self, width=330, fg_color=BG, corner_radius=0)
        side.grid(row=1, column=0, sticky="ns", pady=(10, 0))

        # Estado del registro
        c = self.card(side, "Registro")
        self.lbl_estado = ctk.CTkLabel(c, text="", font=ctk.CTkFont(size=18, weight="bold"), anchor="w")
        self.lbl_estado.pack(fill="x", padx=14)
        self.lbl_tiempo = ctk.CTkLabel(c, text="00:00.0", text_color=TXT, anchor="w",
                                       font=ctk.CTkFont(family="Consolas", size=34, weight="bold"))
        self.lbl_tiempo.pack(fill="x", padx=14)
        self.lbl_muestras = ctk.CTkLabel(c, text="0 muestras", text_color=DIM, anchor="w")
        self.lbl_muestras.pack(fill="x", padx=14, pady=(0, 10))

        # Acciones
        c = self.card(side, "Acciones")
        com = self.cfg.ino.get("comandos", {})

        def boton(txt, cmd, color=CARD2, fg=TXT, estado="normal", parent=c):
            b = ctk.CTkButton(parent, text=txt, command=cmd, fg_color=color, text_color=fg, hover_color=BORDER,
                              border_width=1, border_color=BORDER, state=estado, height=32)
            return b
        if self.controla:
            fila = ctk.CTkFrame(c, fg_color="transparent")
            fila.pack(fill="x", padx=14, pady=(0, 4))
            fila.grid_columnconfigure((0, 1), weight=1)
            b1 = boton("Enganchar · e", lambda: self.puente.enviar("e"), ACCENT, "#ffffff", parent=fila)
            b2 = boton("Soltar · s", lambda: self.puente.enviar("s"), parent=fila)
            b1.grid(row=0, column=0, sticky="ew", padx=(0, 3))
            b2.grid(row=0, column=1, sticky="ew", padx=(3, 0))
        fila = ctk.CTkFrame(c, fg_color="transparent")
        fila.pack(fill="x", padx=14, pady=2)
        fila.grid_columnconfigure((0, 1), weight=1)
        boton("Calibrar IMU · c", self.calibrar, estado="normal" if "c" in com else "disabled", parent=fila
              ).grid(row=0, column=0, sticky="ew", padx=(0, 3))
        boton("Cero encoders · z", self.cero_enc, estado="normal" if "z" in com else "disabled", parent=fila
              ).grid(row=0, column=1, sticky="ew", padx=(3, 0))
        boton("Re-referenciar (cero)", self.tomar_referencia).pack(fill="x", padx=14, pady=2)
        boton("Reiniciar registro", self.reiniciar_registro).pack(fill="x", padx=14, pady=2)
        ctk.CTkButton(c, text="Finalizar y guardar", command=self.finalizar, fg_color=BAD, hover_color="#b82f3c",
                      text_color="#ffffff", height=38, font=ctk.CTkFont(weight="bold")
                      ).pack(fill="x", padx=14, pady=(8, 14))

        # Proporción
        c = self.card(side, "Proporción sensor : Dobot")
        self.lbl_gran = ctk.CTkLabel(c, text="", text_color=ACCENT, font=ctk.CTkFont(size=34, weight="bold"))
        self.lbl_gran.pack(anchor="w", padx=14)
        self.lbl_ejes = [ctk.CTkLabel(c, text="", text_color=DIM, anchor="w", font=ctk.CTkFont(size=12)) for _ in KEYS]
        for l in self.lbl_ejes:
            l.pack(fill="x", padx=14)
        self.lbl_medida = ctk.CTkLabel(c, text="", text_color=TXT, anchor="w", font=ctk.CTkFont(weight="bold"))
        self.lbl_medida.pack(fill="x", padx=14, pady=(6, 12))

        # Opciones de visualización
        c = self.card(side, "Visualización")
        self.sw = {}
        fila = ctk.CTkFrame(c, fg_color="transparent")
        fila.pack(fill="x", padx=14, pady=(0, 4))
        for k in KEYS:
            s = ctk.CTkSwitch(fila, text=NOMBRE[k], progress_color=COLOR[k], command=self.cambiar_ejes,
                              text_color=TXT, width=80)
            if self.cfg.activo(k):
                s.select()
            s.pack(side="left", padx=(0, 8))
            self.sw[k] = s
        self.seg = ctk.CTkSegmentedButton(c, values=["Proporcional", "Zona muerta + límite"],
                                          command=self.cambiar_modelo, selected_color=ACCENT,
                                          unselected_color=CARD2, text_color=TXT,
                                          selected_hover_color=ACCENT, unselected_hover_color=BORDER)
        self.seg.set("Proporcional")
        self.seg.pack(fill="x", padx=14, pady=(4, 2))
        ctk.CTkLabel(c, text="Modelo con el que se calcula el valor esperado del Dobot.", text_color=DIM,
                     font=ctk.CTkFont(size=11), anchor="w", wraplength=290, justify="left"
                     ).pack(fill="x", padx=14, pady=(0, 10))

        # Registro de mensajes
        c = self.card(side, "Mensajes")
        self.txt = ctk.CTkTextbox(c, height=170, fg_color=CARD2, text_color=TXT, wrap="word",
                                  font=ctk.CTkFont(family="Consolas", size=11))
        self.txt.pack(fill="x", padx=10, pady=(0, 10))
        self.txt.configure(state="disabled")

        # ---- Zona principal ----
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.grid(row=1, column=1, sticky="nsew", padx=(0, 12), pady=(10, 10))
        main.grid_rowconfigure(0, weight=1)
        main.grid_columnconfigure(0, weight=1)
        self.tabs = ctk.CTkTabview(main, fg_color=CARD, border_width=1, border_color=BORDER, corner_radius=14,
                                   segmented_button_selected_color=ACCENT, segmented_button_unselected_color=CARD2,
                                   segmented_button_selected_hover_color=ACCENT,
                                   segmented_button_unselected_hover_color=BORDER, text_color=TXT)
        self.tabs.grid(row=0, column=0, sticky="nsew")
        for n in ("Seguimiento", "Sensor y encoders", "Configuración detectada"):
            self.tabs.add(n)

        # Pestaña Seguimiento: tabla abajo, gráficas arriba
        tab = self.tabs.tab("Seguimiento")
        tf = ctk.CTkFrame(tab, fg_color=CARD2, corner_radius=10, border_width=1, border_color=BORDER)
        tf.pack(side="bottom", fill="x", padx=4, pady=(6, 2))
        self.construir_tabla(tf)
        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.graf = GraficaSeguimiento(self.fig, self.cfg)
        self.canvas = FigureCanvasTkAgg(self.fig, master=tab)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Pestaña Sensor y encoders
        self.fig2 = Figure(figsize=(10, 7), dpi=100)
        self.graf2 = GraficaSensor(self.fig2, self.enc_keys)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=self.tabs.tab("Sensor y encoders"))
        self.canvas2.get_tk_widget().pack(fill="both", expand=True)

        # Pestaña Configuración
        self.panel_cfg = ctk.CTkScrollableFrame(self.tabs.tab("Configuración detectada"), fg_color="transparent")
        self.panel_cfg.pack(fill="both", expand=True)
        self.render_config()
        self.actualizar_textos_cfg()
        self._iniciada = True

    def construir_tabla(self, parent):
        st = ttk.Style()
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure("T.Treeview", background=CARD, fieldbackground=CARD, foreground=TXT, rowheight=27,
                     borderwidth=0, font=("Consolas", 10))
        st.configure("T.Treeview.Heading", background=CARD2, foreground=DIM, font=("Segoe UI", 9, "bold"),
                     relief="flat", borderwidth=0)
        st.map("T.Treeview", background=[("selected", CARD)], foreground=[("selected", TXT)])
        cols = [("eje", "Eje", 70, "w"), ("dest", "Destino", 90, "center"),
                ("sabs", "Sensor abs (°)", 105, "e"), ("srel", "Sensor Δ (°)", 100, "e"),
                ("drel", "Dobot Δ (°)", 100, "e"), ("esp", "Esperado (°)", 100, "e"),
                ("err", "Error (°)", 90, "e"), ("rmse", "RMSE", 80, "e"),
                ("mae", "MAE", 80, "e"), ("max", "Máx", 80, "e")]
        self.tree = ttk.Treeview(parent, columns=[c[0] for c in cols], show="headings", height=3,
                                 style="T.Treeview", selectmode="none")
        for cid, txt, w, an in cols:
            self.tree.heading(cid, text=txt, anchor="center" if an != "w" else "w")
            self.tree.column(cid, width=w, anchor=an, stretch=True)
        for k in KEYS:
            self.tree.tag_configure(k, foreground=COLOR[k])
            self.tree.insert("", "end", iid=k, values=(NOMBRE[k],) + ("—",) * 9, tags=(k,))
        self.tree.pack(fill="x", padx=6, pady=6)

    def actualizar_tabla(self):
        c = self.cfg
        for k in KEYS:
            v, n = self.vivo[k], self.nerr[k]
            rmse = math.sqrt(self.suma2[k] / n) if n else NAN
            mae = self.sumaabs[k] / n if n else NAN
            dest = c.nombre_destino(k) + ("" if c.activo(k) else " (off)")
            self.tree.item(k, values=(NOMBRE[k], dest, num(v["sa"], 1, True), num(v["sr"], 2, True),
                                      num(v["dr"], 2, True), num(v["x"], 2, True), num(v["e"], 2, True),
                                      num(rmse), num(mae), num(self.emax[k] if n else NAN)))

    # ---------- pestaña Configuración ----------
    def render_config(self):
        for w in self.panel_cfg.winfo_children():
            w.destroy()
        c, py, ino = self.cfg, self.cfg.py, self.cfg.ino

        def seccion(titulo, archivo):
            f = ctk.CTkFrame(self.panel_cfg, fg_color=CARD2, corner_radius=12, border_width=1, border_color=BORDER)
            f.pack(fill="x", padx=6, pady=6)
            ctk.CTkLabel(f, text=titulo, text_color=TXT, font=ctk.CTkFont(size=15, weight="bold")
                         ).pack(anchor="w", padx=14, pady=(10, 0))
            ctk.CTkLabel(f, text=f"{archivo}\nModificado: {datetime.fromtimestamp(archivo.stat().st_mtime):%Y-%m-%d %H:%M:%S}",
                         text_color=DIM, justify="left", anchor="w", wraplength=900, font=ctk.CTkFont(size=11)
                         ).pack(anchor="w", padx=14, pady=(0, 6))
            g = ctk.CTkFrame(f, fg_color="transparent")
            g.pack(fill="x", padx=10, pady=(0, 10))
            g.grid_columnconfigure(1, weight=1)
            return g

        def filas(g, items):
            for i, (a, b) in enumerate(items):
                ctk.CTkLabel(g, text=a, text_color=DIM, anchor="w", width=190).grid(row=i, column=0, sticky="w", padx=6)
                ctk.CTkLabel(g, text=str(b), text_color=TXT, anchor="w", justify="left", wraplength=700,
                             font=ctk.CTkFont(family="Consolas", size=12, weight="bold")
                             ).grid(row=i, column=1, sticky="w", padx=6)

        per = ino.get("periodo_ms")
        checks = [
            (ino.get("baud") is not None and py["BAUDIOS"] == ino.get("baud"),
             f"Baudios iguales: puente {py['BAUDIOS']} · firmware {ino.get('baud')}"),
            (all(k in ino.get("campos", []) for k in KEYS),
             f"La trama del firmware incluye P, R, Y  (campos: {', '.join(ino.get('campos', [])) or '—'})"),
            (per is not None and py["TIMEOUT_IMU"] > 3 * per / 1000,
             f"TIMEOUT_IMU ({fmt(py['TIMEOUT_IMU'])} s) mayor que 3 periodos de la IMU ({per} ms)"),
            (py["SEND_HZ"] <= 33, f"SEND_HZ = {py['SEND_HZ']}  (Dobot recomienda ≤ 33 Hz para Servo)"),
            (not py["SIMULAR"], "SIMULAR desactivado (el robot recibe comandos)")]
        f = ctk.CTkFrame(self.panel_cfg, fg_color=CARD2, corner_radius=12, border_width=1, border_color=BORDER)
        f.pack(fill="x", padx=6, pady=6)
        ctk.CTkLabel(f, text="Verificaciones de coherencia", text_color=TXT, font=ctk.CTkFont(size=15, weight="bold")
                     ).pack(anchor="w", padx=14, pady=(10, 4))
        if c.reinicio_pendiente:
            ctk.CTkLabel(f, text="⚠  Cambios que requieren reiniciar el visualizador: " + ", ".join(c.reinicio_pendiente),
                         text_color=WARN, anchor="w", wraplength=900).pack(anchor="w", padx=14, pady=2)
        for ok, txt in checks:
            ctk.CTkLabel(f, text=("✔  " if ok else "✖  ") + txt, text_color=OK if ok else BAD, anchor="w",
                         wraplength=900).pack(anchor="w", padx=14, pady=1)
        ctk.CTkLabel(f, text="").pack(pady=2)

        g = seccion("Puente  ·  " + c.ruta_py.name, c.ruta_py)
        filas(g, [("Robot IP", py["ROBOT_IP"]), ("Puerto STM / baudios", f"{py['PUERTO_STM']} @ {py['BAUDIOS']}"),
                  ("Modo", c.modo), ("SIMULAR", py["SIMULAR"]), ("Velocidad global", f"{py['SPEED_FACTOR']} %"),
                  ("Envío", f"{py['SEND_HZ']} Hz"), ("Ir a HOME / confirmar", f"{py['IR_A_HOME']} / {py['CONFIRMAR_HOME']}"),
                  ("HOME (J1..J6)", tuple(py["HOME_JOINTS"])), ("Vel. HOME", f"{py['VEL_HOME']} %"),
                  ("Auto enganchar", py["AUTO_ENGANCHAR"]), ("Tolerancia HOME", f"{py['TOLERANCIA_HOME']}°"),
                  ("Zona muerta", f"{py['ZONA_MUERTA']}°"), ("Límite de desplazamiento", f"±{py['LIMITE_DEG']}°"),
                  ("Paso máximo", f"{py['PASO_MAX_DEG']}° por envío"), ("Timeout IMU", f"{py['TIMEOUT_IMU']} s")])
        g = seccion("Ejes IMU → Dobot", c.ruta_py)
        filas(g, [(f"{NOMBRE[k]}  ({k})", f"→ {c.nombre_destino(k)}   ganancia {fmt(py['EJES'][k]['ganancia'])}"
                   f"   signo {fmt(py['EJES'][k]['signo'])}   {'activo' if c.activo(k) else 'INACTIVO'}") for k in KEYS])
        g = seccion("Firmware  ·  " + c.ruta_ino.name, c.ruta_ino)
        com = ino.get("comandos", {})
        filas(g, [("Baudios", ino.get("baud")), ("Campos de la trama", " · ".join(ino.get("campos", [])) or "—"),
                  ("Periodo de envío", f"{per} ms  ({1000 / per:.1f} Hz)" if per else "—"),
                  ("Comandos por serie", "   ".join(f"'{k}' = {v}" for k, v in com.items()) or "—"),
                  ("IMU", ino.get("imu") or "—"),
                  ("Filtro biquad", f"fc {ino['biquad'][0]:g} Hz @ fs {ino['biquad'][1]:g} Hz" if ino.get("biquad") else "—"),
                  ("Zona muerta del giroscopio", f"{ino['deadband']}°/s" if ino.get("deadband") is not None else "—"),
                  ("EKF  Q_angle / R_angle", f"{ino.get('q_angle')} / {ino.get('r_angle')}"),
                  ("Calibración", f"{ino['cal_n']} muestras" if ino.get("cal_n") else "—"),
                  ("Encoders (timers)", ", ".join(ino.get("enc_timers", [])) or "—"),
                  ("Muestreo de encoders", f"{ino['enc_hz']} Hz" if ino.get("enc_hz") else "—")])

    def actualizar_textos_cfg(self):
        c = self.cfg
        gs = {round(c.ganancia(k), 6) for k in KEYS if c.activo(k)}
        self.lbl_gran.configure(text=f"1 : {fmt(gs.pop())}" if len(gs) == 1 else "por eje")
        for k, l in zip(KEYS, self.lbl_ejes):
            l.configure(text=f"{NOMBRE[k]} → {c.nombre_destino(k)}   Δ Dobot = {fmt(c.ganancia(k))} · Δ Sensor"
                        + ("" if c.activo(k) else "   (inactivo)"))
        self.graf.refrescar_texto()

    # ---------- acciones ----------
    def calibrar(self):
        if self.puente and self.puente.mod is not None and "calibrar" in self.puente.estado:
            self.puente.enviar("c")
        elif self.imu.escribir(b"c"):
            self.log("Comando 'c' enviado al firmware: no muevas el sensor.")
        else:
            self.log("No se pudo calibrar: sin conexión serie.")

    def cero_enc(self):
        self.log("Encoders a cero (comando 'z')." if self.imu.escribir(b"z") else "No se pudo enviar 'z': sin conexión serie.")

    def cambiar_modelo(self, v):
        self.modelo = "prop" if v == "Proporcional" else "zona"

    def cambiar_ejes(self):
        for k in KEYS:
            self.graf.visibles[k] = bool(self.sw[k].get())
        self.graf.leyendas()
        self.canvas.draw_idle()

    def tomar_referencia(self, silencioso=False):
        imu, ei = self.imu.latest()
        fb, ef = self.dobot.latest()
        if imu is not None and ei < 1.0:
            self.ref_imu = dict(imu)
        if fb is not None and ef < 1.0:
            self.ref_fb = {"q": list(fb["q"]), "pose": list(fb["pose"])}
        if not silencioso:
            self.log("Referencia (cero) actualizada.")

    # ---------- muestreo ----------
    def esperado(self, k, s):
        if not math.isfinite(s):
            return NAN
        g, p = self.cfg.ganancia(k), self.cfg.py
        if self.modelo == "zona":
            return limitar(g * zona_muerta(s, p["ZONA_MUERTA"]), -p["LIMITE_DEG"], p["LIMITE_DEG"])
        return g * s

    def tick(self):
        if self.cerrando:
            return
        try:
            self.muestrear(time.perf_counter())
        except Exception:
            traceback.print_exc()
        self.after(int(1000 / GUI_HZ), self.tick)

    def muestrear(self, ahora):
        cfg = self.cfg
        imu, ei = self.imu.latest()
        fb, ef = self.dobot.latest()
        imu_ok, fb_ok = imu is not None and ei < 1.0, fb is not None and ef < 1.0

        # ¿Se registra? Con puente: solo enganchado. Sin puente (demo/monitor): en cuanto hay datos.
        grabar = (self.esta_enganchado() if self.puente else True) and (imu_ok or fb_ok)
        flanco = grabar and not self.grab_prev
        self.grab_prev, self.grabando = grabar, grabar
        if flanco and imu_ok and fb_ok:
            self.tomar_referencia(silencioso=True)          # misma referencia que tu puente al enganchar
            if self.puente:
                self.log("Enganchado: el registro continúa desde el último instante." if self.t
                         else "Enganchado: empieza el registro (t = 0).")
        if imu_ok and self.ref_imu is None:
            self.ref_imu = dict(imu)
        if fb_ok and self.ref_fb is None:
            self.ref_fb = {"q": list(fb["q"]), "pose": list(fb["pose"])}

        # Valores en vivo (siempre, para la tabla)
        campo = "q" if cfg.modo == "JOINT" else "pose"
        for k in KEYS:
            sa = imu[k] if imu_ok else NAN
            s = envolver(imu[k] - self.ref_imu[k]) if imu_ok and self.ref_imu else NAN
            i = cfg.destino(k)
            if fb_ok and self.ref_fb:
                d = fb[campo][i] - self.ref_fb[campo][i]
                d = envolver(d) if (campo == "q" or i >= 3) else d
            else:
                d = NAN
            x = self.esperado(k, s)
            self.vivo[k] = dict(sa=sa, sr=s, dr=d, x=x, e=d - x)

        if not grabar:
            self.t_last = None            # pausa: el tiempo acumulado no avanza
            return

        # Tiempo acumulado (solo cuenta mientras se registra)
        if self.t_last is None:
            self.t_last = ahora
        self.t_acum += ahora - self.t_last
        self.t_last = ahora
        self.t.append(self.t_acum)
        for k in KEYS:
            v = self.vivo[k]
            self.S[k].append(v["sr"])
            self.D[k].append(v["dr"])
            self.X[k].append(v["x"])
            self.E[k].append(v["e"])
            self.A[k].append(v["sa"])
            if math.isfinite(v["e"]):
                self.suma2[k] += v["e"] ** 2
                self.sumaabs[k] += abs(v["e"])
                self.nerr[k] += 1
                self.emax[k] = max(self.emax[k], abs(v["e"]))
        for e in self.enc_keys:
            self.ENC[e].append(imu.get(e, NAN) if imu_ok else NAN)
        self.extra.append((datetime.now(), list(fb["q"]) if fb_ok else [NAN] * 6, int(self.esta_enganchado()), self.modelo))
        self.ticks += 1
        if self.ticks % PLOT_CADA == 0:
            self.redibujar()

    def redibujar(self):
        if len(self.t) < 2:
            return
        tn, n = self.t[-1], len(self.t)
        sl = slice(0, None, max(1, n // MAX_PUNTOS))       # decimación solo visual
        t = np.array(self.t[sl])
        xmax = xmax_acumulativo(tn)
        pestana = self.tabs.get()
        if pestana == "Seguimiento":
            S, D, E = ({k: np.array(d[k][sl]) for k in KEYS} for d in (self.S, self.D, self.E))
            g = self.graf
            g.datos(t, S, D, E)
            g.set_xmax(xmax)
            g.escalar_y(S, D, E)
            g.espera.set_text("")
            g.extra.set_text(f"t = {tn:6.1f} s   ·   {n} muestras")
            self.canvas.draw_idle()
        elif pestana == "Sensor y encoders":
            g = self.graf2
            A = {k: np.array(self.A[k][sl]) for k in KEYS}
            for k in KEYS:
                g.l1[k].set_data(t, A[k])
            ENC = {e: np.array(self.ENC[e][sl]) for e in self.enc_keys}
            for e in self.enc_keys:
                g.l2[e].set_data(t, ENC[e])
            g.a1.set_xlim(0, xmax)
            escalar(g.a1, [A[k] for k in KEYS], 5.0)
            escalar(g.a2, list(ENC.values()), 50.0)
            self.canvas2.draw_idle()

    def proporcion_medida(self):
        if len(self.t) < 10:
            return None
        n = len(self.t)
        i0 = next((i for i in range(n - 1, -1, -1) if self.t[i] < self.t[-1] - 10.0), -1) + 1
        numer = den = 0.0
        for k in KEYS:
            if not (self.graf.visibles[k] and self.cfg.activo(k)):
                continue
            s, d = np.array(self.S[k][i0:]), np.array(self.D[k][i0:])
            m = np.isfinite(s) & np.isfinite(d)
            numer += float(np.sum(s[m] * d[m]))
            den += float(np.sum(s[m] ** 2))
        return numer / den if den > 25.0 else None

    # ---------- panel ----------
    def refrescar_panel(self):
        if self.cerrando:
            return
        c = self.cfg
        if time.time() - self.t_cfg > REVISAR_CFG_S:      # recarga de tus archivos
            self.t_cfg = time.time()
            if c.revisar():
                self.log("Cambios detectados en tus archivos .py / .ino.")
                self.render_config()
            if c.ajustes_pendientes and not self.esta_enganchado():
                c.aplicar_ajustes(self.puente.mod if self.puente and self.puente.mod else None)
                self.actualizar_textos_cfg()
                self.render_config()
                self.log("Parámetros de ajuste actualizados desde tu puente.")
        if self.puente and self.puente.terminado and not self.puente.detener and self.lector_imu is None:
            self.log("El puente terminó: la IMU se sigue leyendo en modo monitor.")
            self.iniciar_lector_imu()

        # Chips del encabezado
        hz, esp = self.imu.hz(), c.ino.get("periodo_ms")
        if self.imu.conectado:
            self.set_chip(self.ch_imu, f"Sensor  {hz:4.1f} Hz" + (f" / {1000 / esp:.0f}" if esp else ""),
                          "ok" if hz > 0 else "warn")
        else:
            self.set_chip(self.ch_imu, "Sensor  sin señal", "bad")
        self.set_chip(self.ch_dob, "Dobot  conectado" if self.dobot.conectado else "Dobot  sin señal",
                      "ok" if self.dobot.conectado else "bad")
        if self.puente:
            st = "Enganchado" if self.esta_enganchado() else ("Esperando puente" if not self.puente.mod else "Suelto")
            if self.puente.terminado:
                st = "Puente finalizado"
            self.set_chip(self.ch_ctl, f"Control  {st}", "ok" if self.esta_enganchado() else "warn")
        else:
            self.set_chip(self.ch_ctl, "Control  desactivado", "warn")
        self.set_chip(self.ch_modo, "DEMO" if self.demo else ("CONTROL" if self.controla else "MONITOR"), "info")
        dur = self.t_acum
        self.set_chip(self.ch_t, ("● REC  " if self.grabando else "❚❚  ") + f"{int(dur // 60):02d}:{int(dur % 60):02d}",
                      "ok" if self.grabando else "warn")

        # Tarjeta de registro
        if self.grabando:
            self.lbl_estado.configure(text="● Registrando", text_color=OK)
        elif self.puente:
            self.lbl_estado.configure(text="Esperando enganche…" if not self.t else "Pausado (suelto)", text_color=WARN)
        else:
            self.lbl_estado.configure(text="Esperando datos…", text_color=WARN)
        self.lbl_tiempo.configure(text=f"{int(dur // 60):02d}:{dur % 60:04.1f}")
        self.lbl_muestras.configure(text=f"{len(self.t)} muestras")

        m = self.proporcion_medida()
        self.lbl_medida.configure(text="Proporción medida:  " + ("—" if m is None else f"1 : {m:.3f}"))
        self.actualizar_tabla()

        # Mensaje sobre la gráfica mientras no hay datos
        msg = "" if len(self.t) >= 2 else (
            "Esperando enganche…\nPulsa «Enganchar · e»\nEl tiempo empieza en 0 al engancharse" if self.puente
            else "Esperando datos…")
        if self.graf.espera.get_text() != msg:
            self.graf.espera.set_text(msg)
            self.canvas.draw_idle()

        while True:
            try:
                texto = self.cola_log.get_nowait()
            except queue.Empty:
                break
            self.txt.configure(state="normal")
            self.txt.insert("end", texto + "\n")
            self.txt.see("end")
            self.txt.configure(state="disabled")
        self.after(250, self.refrescar_panel)

    # ---------- exportación ----------
    def guardar(self):
        if not self.t:
            return None
        fin = datetime.now()
        marca = fin.strftime("%Y-%m-%d_%H-%M-%S")
        carpeta = carpeta_salida() / f"Prueba_{marca}"      # una carpeta por prueba con sus dos archivos
        carpeta.mkdir(parents=True, exist_ok=True)
        png, csv_path = carpeta / f"image_{marca}.png", carpeta / f"data_{marca}.csv"

        fig = Figure(figsize=(16, 9), dpi=150)
        g = GraficaSeguimiento(fig, self.cfg)
        g.visibles = dict(self.graf.visibles)
        g.leyendas()
        g.espera.set_text("")
        t = np.array(self.t)
        S, D, E = ({k: np.array(d[k]) for k in KEYS} for d in (self.S, self.D, self.E))
        g.datos(t, S, D, E)
        g.set_xmax(max(float(t[-1]), 1e-3))
        g.escalar_y(S, D, E)
        rm = " · ".join(f"{NOMBRE[k]} {math.sqrt(self.suma2[k] / self.nerr[k]):.2f}°" if self.nerr[k] else f"{NOMBRE[k]} —"
                        for k in KEYS)
        g.extra.set_text(f"Finalizado: {fin:%Y-%m-%d %H:%M:%S}  ·  {t[-1]:.1f} s  ·  {len(t)} muestras\nRMSE: {rm}")
        fig.savefig(png, facecolor=fig.get_facecolor())

        c, nom = self.cfg, [NOMBRE[k].lower() for k in KEYS]
        cab = (["t_s", "fecha_hora"] + [f"sensor_{n}_abs_deg" for n in nom] + list(self.enc_keys)
               + [f"sensor_{n}_rel_deg" for n in nom] + [f"dobot_J{i}_abs_deg" for i in range(1, 7)]
               + [f"dobot_{n}_{c.nombre_destino(k)}_rel_deg" for n, k in zip(nom, KEYS)]
               + [f"esperado_{n}_deg" for n in nom] + [f"error_{n}_deg" for n in nom] + ["enganchado", "modelo_error"])
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(cab)
            for i, (hora, q, eng, modelo) in enumerate(self.extra):
                w.writerow([f"{self.t[i]:.4f}", hora.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]]
                           + [f"{self.A[k][i]:.4f}" for k in KEYS] + [f"{self.ENC[e][i]:g}" for e in self.enc_keys]
                           + [f"{self.S[k][i]:.4f}" for k in KEYS] + [f"{v:.4f}" for v in q]
                           + [f"{self.D[k][i]:.4f}" for k in KEYS] + [f"{self.X[k][i]:.4f}" for k in KEYS]
                           + [f"{self.E[k][i]:.4f}" for k in KEYS] + [eng, modelo])
        return png, csv_path

    def detener_hilos(self):
        for h in self.hilos:
            if isinstance(h, PuenteExterno):
                h.parar()
            else:
                h.activo = False
        if self.puente:
            self.puente.join(1.5)

    def finalizar(self):
        if self.cerrando:
            return
        self.cerrando = True
        self.detener_hilos()
        rutas = None
        try:
            rutas = self.guardar()
            self.guardado = True
        except Exception:
            traceback.print_exc()
            messagebox.showerror("Error al guardar", traceback.format_exc(), parent=self)
        if rutas:
            print(f"Imagen: {rutas[0]}\nDatos : {rutas[1]}")
            messagebox.showinfo("Datos guardados", f"Carpeta:\n{rutas[0].parent}\n\nImagen:\n{rutas[0].name}\n\nDatos:\n{rutas[1].name}", parent=self)
        else:
            print("No hubo datos que guardar.")
        self.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="datos simulados con tus parametros, sin hardware")
    ap.add_argument("--monitor", action="store_true", help="solo lectura: no ejecuta el puente")
    ap.add_argument("--puente", help="ruta de Comunicacion_STM_CR3.py")
    ap.add_argument("--ino", help="ruta del .ino del firmware")
    ap.add_argument("--puerto", help="fuerza el puerto serie (sobre el del puente)")
    ap.add_argument("--ip", help="fuerza la IP del robot (sobre la del puente)")
    args = ap.parse_args()
    ruta_py, ruta_ino = buscar_archivos(args.puente or RUTA_PUENTE, args.ino or RUTA_INO)
    ov = {k: v for k, v in (("PUERTO_STM", args.puerto), ("ROBOT_IP", args.ip)) if v}
    cfg = Config(ruta_py, ruta_ino, ov)
    print(f"Puente  : {ruta_py}\nFirmware: {ruta_ino}")
    app = App(args, cfg)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if not app.cerrando:
            app.detener_hilos()
            try:
                rutas = app.guardar()
                if rutas:
                    print(f"Imagen: {rutas[0]}\nDatos : {rutas[1]}")
            except Exception:
                traceback.print_exc()


if __name__ == "__main__":
    main()