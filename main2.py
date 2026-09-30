# Actividad 6 - Punto 1: ESP32 (MicroPython) - teclado virtual desde el PC + LCD I2C (SDA=21, SCL=22)

import sys
import time
import uselect
from machine import I2C, Pin

SDA_PIN = 21
SCL_PIN = 22
DIR_LCD = 0x27
TIMEOUT_MS = 60000

# ------------------ Driver mínimo de LCD I2C (HD44780 + PCF8574) ------------------
class LCD:

    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.luz = 0x08
        time.sleep_ms(50)
        for _ in range(3):
            self._nibble(0x30)
            time.sleep_ms(5)
        self._nibble(0x20)
        self.comando(0x28)
        self.comando(0x0C)
        self.comando(0x06)
        self.limpiar()

    def _escribir(self, b):
        self.i2c.writeto(self.addr, bytes([b | self.luz]))

    def _nibble(self, b, rs=0):
        self._escribir(b | rs | 0x04)
        time.sleep_us(1)
        self._escribir(b | rs)
        time.sleep_us(50)

    def _byte(self, b, rs):
        self._nibble(b & 0xF0, rs)
        self._nibble((b << 4) & 0xF0, rs)

    def comando(self, c):
        self._byte(c, 0)
        if c in (0x01, 0x02):
            time.sleep_ms(2)

    def limpiar(self):
        self.comando(0x01)

    def escribir(self, fila, texto):
        self.comando(0x80 | (0x40 if fila else 0x00))
        for ch in texto[:16]:
            self._byte(ord(ch), 1)

# ------------------ Inicialización ------------------
i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
dispositivos = i2c.scan()
print("# I2C encontrados:", [hex(d) for d in dispositivos])

lcd = None
if DIR_LCD in dispositivos:
    lcd = LCD(i2c, DIR_LCD)
else:
    print("# LCD no encontrada, solo se mostrará en la terminal")

def mostrar(l1, l2=""):
    if lcd:
        lcd.limpiar()
        lcd.escribir(0, l1)
        lcd.escribir(1, l2)
    print("# LCD | {} | {}".format(l1, l2))

# ------------------ Lógica del "teclado" ------------------
digito_sel = None
ocupado = False
t_envio = 0

def enviar(msg):
    global ocupado, t_envio
    print(msg)
    ocupado = True
    t_envio = time.ticks_ms()

def procesar_tecla(k):
    global digito_sel
    if ocupado:
        mostrar("Dibujando...", "Espere por favor")
        return

    if k.isdigit():
        digito_sel = k
        mostrar("Digito elegido:", k + " (Enter=env)")
    elif k == "#":
        if digito_sel is None:
            mostrar("Primero elija", "un digito 0-9")
            return
        enviar("DIG:" + digito_sel)
        mostrar("Dibujando el", digito_sel + " en PyBullet")
    elif k == "*":
        digito_sel = None
        mostrar("Seleccion borrada", "Pulse un digito")
    elif k == "A":
        enviar("HOME")
        mostrar("Brazo a HOME", "...")
    elif k == "C":
        enviar("CLEAR")
        mostrar("Limpiando", "pizarra...")
    else:
        mostrar("Tecla sin uso:", k)

def procesar_linea(linea):
    global ocupado, digito_sel
    if linea.startswith("K:") and len(linea) >= 3:
        procesar_tecla(linea[2].upper())
    elif linea == "OK" and ocupado:
        ocupado = False
        digito_sel = None
        mostrar("Listo!", "Otro digito?")

# ------------------ Bucle principal ------------------
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)
buffer = ""

mostrar("Brazo dibujante", "Pulse un digito")
print("# ESP32 lista")

while True:
    if poll.poll(10):
        c = sys.stdin.read(1)
        if c in ("\n", "\r"):
            if buffer:
                procesar_linea(buffer.strip())
            buffer = ""
        else:
            buffer += c

    if ocupado and time.ticks_diff(time.ticks_ms(), t_envio) > TIMEOUT_MS:
        ocupado = False
        mostrar("Sin respuesta", "del PC :(")
