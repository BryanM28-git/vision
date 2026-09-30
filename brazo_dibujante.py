# Actividad 6 - Punto 1: brazo robótico en PyBullet que dibuja el dígito recibido de la ESP32
# Uso: python brazo_dibujante.py --puerto COM3 --urdf brazo.urdf

\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\

import argparse
import math
import os
import queue
import threading
import time

import numpy as np
import pybullet as p
import pybullet_data

try:
    import serial
except ImportError:
    serial = None
try:
    import cv2
except ImportError:
    cv2 = None

# ------------------ Parámetros de la pizarra (plano vertical frente al robot) ------------------
PLANO_X = 0.35
CENTRO_Y = 0.0
CENTRO_Z = 0.30
ESCALA = 0.15
RETIRO = 0.05
PASO = 0.004
PASOS_SIM = 8
ESPEJAR_Y = False

# ------------------ Trazos de cada dígito (caja u: 0..0.6, v: 0..1) ------------------
def elipse(cx, cy, rx, ry, a0, a1, n=40):
    return [(cx + rx * math.cos(math.radians(a)), cy + ry * math.sin(math.radians(a)))
            for a in np.linspace(a0, a1, n)]

DIGITOS = {
    "0": [elipse(0.30, 0.50, 0.28, 0.50, 90, 450)],
    "1": [[(0.15, 0.80), (0.35, 1.00), (0.35, 0.00)], [(0.15, 0.00), (0.55, 0.00)]],
    "2": [[(0.05, 0.75)] + elipse(0.30, 0.72, 0.25, 0.26, 160, -30) + [(0.03, 0.00), (0.60, 0.00)]],
    "3": [elipse(0.28, 0.75, 0.24, 0.24, 150, -90) + elipse(0.28, 0.26, 0.27, 0.26, 90, -150)],
    "4": [[(0.45, 0.00), (0.45, 1.00), (0.00, 0.30), (0.60, 0.30)]],
    "5": [[(0.55, 1.00), (0.10, 1.00), (0.06, 0.58)] + elipse(0.30, 0.32, 0.27, 0.28, 140, -150)],
    "6": [[(0.50, 0.95), (0.30, 1.00), (0.12, 0.88), (0.03, 0.60)] + elipse(0.30, 0.30, 0.27, 0.30, 180, 540)],
    "7": [[(0.00, 1.00), (0.60, 1.00), (0.20, 0.00)]],
    "8": [elipse(0.30, 0.76, 0.20, 0.24, -90, 270) + elipse(0.30, 0.27, 0.26, 0.25, 90, 450)],
    "9": [elipse(0.30, 0.72, 0.25, 0.26, 0, 360) + [(0.50, 0.00)]],
}

# ------------------ Brazo de ejemplo (por si no hay URDF) ------------------
URDF_EJEMPLO = """<?xml version="1.0"?>
<robot name="brazo_dibujante">
  <material name="gris"><color rgba="0.6 0.6 0.6 1"/></material>
  <material name="azul"><color rgba="0.4 0.75 0.95 1"/></material>
  <material name="naranja"><color rgba="0.95 0.65 0.2 1"/></material>
  <material name="blanco"><color rgba="1 1 1 1"/></material>
  <material name="negro"><color rgba="0.1 0.1 0.1 1"/></material>

  <link name="base">
    <visual><origin xyz="0 0 0.025"/><geometry><cylinder radius="0.15" length="0.05"/></geometry><material name="gris"/></visual>
    <inertial><mass value="2"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="j1_base" type="revolute">
    <parent link="base"/><child link="eslabon1"/><origin xyz="0 0 0.05"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <link name="eslabon1">
    <visual><origin xyz="0 0 0.125"/><geometry><cylinder radius="0.03" length="0.25"/></geometry><material name="azul"/></visual>
    <inertial><origin xyz="0 0 0.125"/><mass value="0.5"/><inertia ixx="0.003" iyy="0.003" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="j2_hombro" type="revolute">
    <parent link="eslabon1"/><child link="eslabon2"/><origin xyz="0 0 0.25"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="3"/>
  </joint>
  <link name="eslabon2">
    <visual><origin xyz="0 0 0.125"/><geometry><cylinder radius="0.025" length="0.25"/></geometry><material name="naranja"/></visual>
    <inertial><origin xyz="0 0 0.125"/><mass value="0.4"/><inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="j3_codo" type="revolute">
    <parent link="eslabon2"/><child link="eslabon3"/><origin xyz="0 0 0.25"/><axis xyz="0 1 0"/>
    <limit lower="-2.6" upper="2.6" effort="50" velocity="3"/>
  </joint>
  <link name="eslabon3">
    <visual><origin xyz="0 0 0.10"/><geometry><box size="0.04 0.04 0.20"/></geometry><material name="blanco"/></visual>
    <inertial><origin xyz="0 0 0.10"/><mass value="0.2"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="j_lapiz" type="fixed">
    <parent link="eslabon3"/><child link="lapiz"/><origin xyz="0 0 0.20"/>
  </joint>
  <link name="lapiz">
    <visual><origin xyz="0 0 0.025"/><geometry><cylinder radius="0.008" length="0.05"/></geometry><material name="negro"/></visual>
    <inertial><mass value="0.01"/><inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="j_punta" type="fixed">
    <parent link="lapiz"/><child link="punta"/><origin xyz="0 0 0.05"/>
  </joint>
  <link name="punta">
    <inertial><mass value="0.001"/><inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
</robot>
"""

# ------------------ Utilidades ------------------
def uv_a_xyz(u, v, retiro=0.0):
    """Convierte un punto de la caja del dígito a coordenadas del mundo."""
    signo = 1 if ESPEJAR_Y else -1
    y = CENTRO_Y + signo * (u - 0.3) * ESCALA
    z = CENTRO_Z + (v - 0.5) * ESCALA
    return [PLANO_X - retiro, y, z]

def interpolar(trazo):
    paso_uv = PASO / ESCALA
    puntos = []
    for a, b in zip(trazo[:-1], trazo[1:]):
        k = max(1, int(math.dist(a, b) / paso_uv))
        for i in range(1, k + 1):
            t = i / k
            puntos.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return puntos

class BrazoDibujante:
    def __init__(self, urdf, efector):
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        if not urdf or not os.path.exists(urdf):
            if urdf:
                print(f"[!] No encontré '{urdf}', uso el brazo de ejemplo.")
            urdf = "brazo_ejemplo.urdf"
            with open(urdf, "w") as f:
                f.write(URDF_EJEMPLO)

        self.robot = p.loadURDF(urdf, [0, 0, 0], useFixedBase=True)
        n = p.getNumJoints(self.robot)
        self.moviles = [j for j in range(n) if p.getJointInfo(self.robot, j)[2] != p.JOINT_FIXED]

        self.ee = n - 1
        if efector:
            for j in range(n):
                if p.getJointInfo(self.robot, j)[12].decode() == efector:
                    self.ee = j
        print("Articulaciones móviles:", [p.getJointInfo(self.robot, j)[1].decode() for j in self.moviles])
        print("Efector:", p.getJointInfo(self.robot, self.ee)[12].decode())

        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.003, ESCALA, ESCALA * 0.9], rgbaColor=[1, 1, 1, 1])
        p.createMultiBody(0, -1, vis, [PLANO_X + 0.005, CENTRO_Y, CENTRO_Z])
        p.resetDebugVisualizerCamera(1.0, -90, -15, [PLANO_X / 2, CENTRO_Y, CENTRO_Z])

        self.ultimo = None
        self.limpiar()
        self.home()

    # ---------- movimiento ----------
    def punta(self):
        return p.getLinkState(self.robot, self.ee, computeForwardKinematics=True)[4]

    def mover_a(self, destino, dibujar, pasos=PASOS_SIM):
        q = p.calculateInverseKinematics(self.robot, self.ee, destino,
                                         maxNumIterations=200, residualThreshold=1e-5)
        p.setJointMotorControlArray(self.robot, self.moviles, p.POSITION_CONTROL,
                                    targetPositions=q, forces=[200] * len(self.moviles))
        for _ in range(pasos):
            p.stepSimulation()
            time.sleep(1 / 240)

        pos = self.punta()
        if dibujar and self.ultimo is not None:
            p.addUserDebugLine(self.ultimo, pos, [0, 0, 0], lineWidth=4, lifeTime=0)
        self.ultimo = pos if dibujar else None

    def home(self):
        objetivo = uv_a_xyz(0.3, 0.5, retiro=3 * RETIRO)
        for _ in range(4):
            self.mover_a(objetivo, dibujar=False, pasos=60)
        error = math.dist(objetivo, self.punta())
        print(f"HOME -> punta en {np.round(self.punta(), 3)}, error = {error*100:.1f} cm")
        if error > 0.02:
            print("[!] El brazo no llega al objetivo: revisa --efector o ajusta --plano-x / --centro-z")

    # ---------- pizarra ----------
    def limpiar(self):
        p.removeAllUserDebugItems()
        if cv2 is not None:
            self.lienzo = np.full((520, 420, 3), 255, np.uint8)
            cv2.rectangle(self.lienzo, (0, 0), (419, 519), (0, 0, 0), 20)
            cv2.imshow("Pizarra", self.lienzo)
            cv2.waitKey(1)

    def pintar(self, a, b):
        if cv2 is None:
            return
        pa = (int(90 + a[0] * 400), int(460 - a[1] * 400))
        pb = (int(90 + b[0] * 400), int(460 - b[1] * 400))
        cv2.line(self.lienzo, pa, pb, (0, 0, 0), 6, cv2.LINE_AA)
        cv2.imshow("Pizarra", self.lienzo)
        cv2.waitKey(1)

    def dibujar_digito(self, d):
        if d not in DIGITOS:
            print("Dígito no válido:", d)
            return
        print(f"Dibujando {d}...")
        self.limpiar()
        p.addUserDebugText(f"Digito: {d}", [PLANO_X, CENTRO_Y, CENTRO_Z + ESCALA * 0.8],
                           textColorRGB=[1, 0, 0], textSize=1.5)

        for trazo in DIGITOS[d]:
            u0, v0 = trazo[0]
            self.mover_a(uv_a_xyz(u0, v0, RETIRO), False, 80)
            self.mover_a(uv_a_xyz(u0, v0), False, 40)
            self.ultimo = self.punta()
            anterior = (u0, v0)
            for u, v in interpolar(trazo):
                self.mover_a(uv_a_xyz(u, v), True)
                self.pintar(anterior, (u, v))
                anterior = (u, v)
            self.mover_a(uv_a_xyz(*anterior, RETIRO), False, 40)
        self.home()
        print("Terminado.")

# ------------------ Hilos de entrada ------------------
def hilo_serial(ser, cola):
    while True:
        try:
            linea = ser.readline().decode(errors="ignore").strip()
            if linea:
                cola.put(linea)
        except Exception as e:
            print("Error serial:", e)
            break

ENTER = (10, 13, getattr(p, "B3G_RETURN", -1))
BORRAR = (8, 127, getattr(p, "B3G_BACKSPACE", -1), getattr(p, "B3G_DELETE", -1))

def traducir(cod):
    """Convierte un código de tecla del PC en una tecla del 'teclado 4x4'."""
    if cod in ENTER:
        return "#"
    if cod in BORRAR:
        return "*"
    if 0 <= cod < 256:
        c = chr(cod).upper()
        if c.isdigit() or c in "#*":
            return c
        if c == "H":
            return "A"
        if c == "C":
            return "C"
    return None

def leer_teclas():
    """Lee el teclado desde la ventana de PyBullet y desde la ventana 'Pizarra'."""
    teclas = []
    for cod, estado in p.getKeyboardEvents().items():
        if estado & p.KEY_WAS_TRIGGERED:
            teclas.append(traducir(cod))
    if cv2 is not None:
        cod = cv2.waitKey(1)
        if cod != -1:
            teclas.append(traducir(cod & 0xFF))
    return [t for t in teclas if t]

# ------------------ Programa principal ------------------
def main():
    global PLANO_X, CENTRO_Z, ESCALA
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", default="COM3", help="puerto de la ESP32 (COM3, /dev/ttyUSB0...)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--urdf", default=None, help="ruta al URDF del brazo")
    ap.add_argument("--efector", default=None, help="nombre del link de la punta")
    ap.add_argument("--plano-x", type=float, default=PLANO_X)
    ap.add_argument("--centro-z", type=float, default=CENTRO_Z)
    ap.add_argument("--escala", type=float, default=ESCALA)
    args = ap.parse_args()
    PLANO_X, CENTRO_Z, ESCALA = args.plano_x, args.centro_z, args.escala

    cola = queue.Queue()
    ser = None
    if serial is not None:
        try:
            ser = serial.Serial(args.puerto, args.baud, timeout=0.1)
            time.sleep(2)
            print(f"Conectado a {args.puerto}")
            threading.Thread(target=hilo_serial, args=(ser, cola), daemon=True).start()
        except Exception as e:
            print(f"[!] No pude abrir {args.puerto} ({e}).")
            ser = None
    if ser is None:
        print("Modo prueba sin ESP32: el PC procesa las teclas directamente.")

    brazo = BrazoDibujante(args.urdf, args.efector)
    print("Listo. Selecciona la ventana de PyBullet y pulsa un dígito + Enter.")

    def responder():
        if ser is not None:
            ser.write(b"OK\n")

    seleccion = {"d": None}

    def manejar_tecla(t):
        print("Tecla detectada:", t)
        if ser is not None:
            ser.write(f"K:{t}\n".encode())
            return
        if t.isdigit():
            seleccion["d"] = t
            print("Dígito elegido:", t)
        elif t == "#" and seleccion["d"]:
            cola.put("DIG:" + seleccion["d"])
            seleccion["d"] = None
        elif t == "*":
            seleccion["d"] = None
        elif t == "A":
            cola.put("HOME")
        elif t == "C":
            cola.put("CLEAR")

    while p.isConnected():
        for t in leer_teclas():
            manejar_tecla(t)

        try:
            cmd = cola.get_nowait()
        except queue.Empty:
            cmd = None

        if cmd:
            if cmd.startswith("#"):
                print("ESP32:", cmd[1:].strip())
            elif cmd.startswith("DIG:"):
                brazo.dibujar_digito(cmd[4:].strip())
                responder()
            elif cmd == "HOME":
                brazo.home()
                responder()
            elif cmd == "CLEAR":
                brazo.limpiar()
                responder()
            else:
                print("ESP32 (otro):", cmd)
        else:
            p.stepSimulation()
            time.sleep(1 / 240)

if __name__ == "__main__":
    main()
