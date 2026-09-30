# Actividad 6 - Punto 2: ESP32 (MicroPython) que simula maestro y esclavo SPI
# Puente GPIO23 (MOSI) -> GPIO19 (MISO) | LCD I2C: SDA=21, SCL=22

import sys
import time
import uselect
from machine import I2C, SPI, Pin

DIR_LCD = 0x27
CABECERA = 0xAA

# ------------------ Driver mínimo de LCD I2C (el mismo del punto 1) ------------------
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

# ------------------ ESP-B: esclavo (muestra en la LCD) ------------------
class Esclavo:
    def __init__(self):
        i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
        self.lcd = LCD(i2c, DIR_LCD) if DIR_LCD in i2c.scan() else None
        if self.lcd is None:
            print("[ESP-B] LCD no encontrada, solo se muestra en la terminal")
        self.mostrar("Esclavo SPI", "Esperando dato..")

    def mostrar(self, l1, l2=""):
        if self.lcd:
            self.lcd.limpiar()
            self.lcd.escribir(0, l1)
            self.lcd.escribir(1, l2)
        print("[ESP-B] LCD | {} | {}".format(l1, l2))

    def recibir(self, trama):
        """Valida la trama que llegó por MISO y la muestra."""
        ok = (len(trama) == 4 and trama[0] == CABECERA
              and trama[3] == (trama[0] ^ trama[1] ^ trama[2])
              and trama[1] <= 9)
        if ok:
            self.mostrar("Digito recibido:", "   {}    ({}%)".format(trama[1], trama[2]))
        else:
            self.mostrar("Error de trama", " ".join("{:02X}".format(b) for b in trama))
            print("[ESP-B] Revisa el puente GPIO23 -> GPIO19")

# ------------------ ESP-A: maestro (envía por SPI) ------------------
class Maestro:
    def __init__(self, esclavo):
        self.spi = SPI(2, baudrate=100000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
                       sck=Pin(18), mosi=Pin(23), miso=Pin(19))
        self.cs = Pin(5, Pin.OUT, value=1)
        self.esclavo = esclavo

    def enviar(self, digito, confianza):
        trama = bytes([CABECERA, digito, confianza, CABECERA ^ digito ^ confianza])
        recibido = bytearray(4)

        self.cs(0)
        self.spi.write_readinto(trama, recibido)
        self.cs(1)

        print("[ESP-A] SPI enviado  :", " ".join("{:02X}".format(b) for b in trama))
        print("[ESP-A] SPI en MISO  :", " ".join("{:02X}".format(b) for b in recibido))
        self.esclavo.recibir(recibido)

    def procesar_linea(self, linea):

        if not linea.startswith("D:"):
            return
        try:
            partes = linea[2:].split(",")
            digito = int(partes[0])
            confianza = int(partes[1]) if len(partes) > 1 else 0
        except ValueError:
            print("[ESP-A] Mensaje invalido:", linea)
            return
        if 0 <= digito <= 9:
            self.enviar(digito, max(0, min(100, confianza)))

# ------------------ Programa principal ------------------
esclavo = Esclavo()
maestro = Maestro(esclavo)
print("[ESP-A] Maestro SPI listo, esperando datos del PC")

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)
buffer = ""

while True:
    if poll.poll(10):
        c = sys.stdin.read(1)
        if c in ("\n", "\r"):
            if buffer:
                maestro.procesar_linea(buffer.strip())
            buffer = ""
        else:
            buffer += c
