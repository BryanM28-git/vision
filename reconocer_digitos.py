# Actividad 6 - Punto 2: cámara -> OpenCV -> CNN -> serial a la ESP32
# Uso: python reconocer_digitos.py --puerto COM3   (Espacio = enviar, q = salir)

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
import time
from collections import deque

import cv2
import numpy as np
from tensorflow.keras.models import load_model

try:
    import serial
except ImportError:
    serial = None

LADO_ROI = 260
AREA_MIN = 150
CONF_MIN = 0.85
CUADROS_ESTABLES = 12

def preprocesar(roi):
    """Convierte el recorte de la cámara en una imagen 28x28 estilo MNIST."""
    gris = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gris = cv2.GaussianBlur(gris, (5, 5), 0)

    binaria = cv2.adaptiveThreshold(gris, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, 31, 10)
    binaria = cv2.morphologyEx(binaria, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    binaria = cv2.dilate(binaria, np.ones((3, 3), np.uint8), iterations=2)

    contornos, _ = cv2.findContours(binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos = [c for c in contornos if cv2.contourArea(c) > AREA_MIN]
    if not contornos:
        return None, binaria, None

    x, y, w, h = cv2.boundingRect(np.vstack(contornos))
    digito = binaria[y:y + h, x:x + w]

    lado = max(w, h)
    cuadro = np.zeros((lado, lado), np.uint8)
    oy, ox = (lado - h) // 2, (lado - w) // 2
    cuadro[oy:oy + h, ox:ox + w] = digito
    img = np.pad(cv2.resize(cuadro, (20, 20), interpolation=cv2.INTER_AREA), 4)

    m = cv2.moments(img)
    if m["m00"]:
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        img = cv2.warpAffine(img, np.float32([[1, 0, 14 - cx], [0, 1, 14 - cy]]), (28, 28))
    return img, binaria, (x, y, w, h)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", default="COM4", help="puerto de la ESP32 maestro")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camara", type=int, default=0)
    ap.add_argument("--modelo", default="modelo_digitos.keras")
    args = ap.parse_args()

    modelo = load_model(args.modelo)
    print("Modelo cargado.")

    ser = None
    if serial is not None:
        try:
            ser = serial.Serial(args.puerto, args.baud, timeout=0)
            time.sleep(2)
            print(f"Conectado a la ESP32 maestro en {args.puerto}")
        except Exception as e:
            print(f"[!] No pude abrir {args.puerto} ({e}). Solo se mostrará en pantalla.")

    cam = cv2.VideoCapture(args.camara)
    if not cam.isOpened():
        raise SystemExit("No pude abrir la cámara")

    historial = deque(maxlen=CUADROS_ESTABLES)
    ultimo_enviado = None

    def enviar(d, conf):
        nonlocal ultimo_enviado
        msg = f"D:{d},{int(conf * 100)}\n"
        if ser is not None:
            ser.write(msg.encode())
        print("Enviado ->", msg.strip())
        ultimo_enviado = d

    while True:
        ok, cuadro = cam.read()
        if not ok:
            break
        cuadro = cv2.flip(cuadro, 1)
        alto, ancho = cuadro.shape[:2]
        x0, y0 = (ancho - LADO_ROI) // 2, (alto - LADO_ROI) // 2
        roi = cuadro[y0:y0 + LADO_ROI, x0:x0 + LADO_ROI]

        img28, binaria, caja = preprocesar(roi)
        digito, conf = None, 0.0
        if img28 is not None:
            prob = modelo.predict(img28.reshape(1, 28, 28, 1).astype("float32") / 255.0,
                                  verbose=0)[0]
            digito, conf = int(np.argmax(prob)), float(np.max(prob))
            x, y, w, h = caja
            cv2.rectangle(roi, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(cuadro, f"Numero: {digito} ({conf*100:.1f}%)", (x0, y0 - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Entrada CNN (28x28)", cv2.resize(img28, (140, 140),
                                                          interpolation=cv2.INTER_NEAREST))

        historial.append(digito if conf >= CONF_MIN else None)
        if (len(historial) == CUADROS_ESTABLES and historial[0] is not None
                and historial.count(historial[0]) == CUADROS_ESTABLES
                and historial[0] != ultimo_enviado):
            enviar(historial[0], conf)

        if len(historial) == CUADROS_ESTABLES and historial.count(None) == CUADROS_ESTABLES:
            ultimo_enviado = None

        cv2.rectangle(cuadro, (x0, y0), (x0 + LADO_ROI, y0 + LADO_ROI), (0, 255, 0), 2)
        cv2.imshow("Camara", cuadro)
        cv2.imshow("Binaria", binaria)

        if ser is not None and ser.in_waiting:
            for linea in ser.read(ser.in_waiting).decode(errors="ignore").splitlines():
                if linea.strip():
                    print("ESP-A:", linea.strip())

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            break
        if tecla == ord(" ") and digito is not None:
            enviar(digito, conf)

    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
