# Actividad 6 - Punto 2: entrenamiento de la CNN con MNIST -> modelo_digitos.keras

\
\
\
\
\
\
\
\

import tensorflow as tf
from tensorflow.keras import layers, models

# ---------------- Datos ----------------
(x_ent, y_ent), (x_pru, y_pru) = tf.keras.datasets.mnist.load_data()
x_ent = x_ent[..., None].astype("float32") / 255.0
x_pru = x_pru[..., None].astype("float32") / 255.0

# ---------------- Aumento de datos ----------------

aumento = models.Sequential([
    layers.RandomRotation(0.08),
    layers.RandomZoom(0.10),
    layers.RandomTranslation(0.10, 0.10),
])

# ---------------- Red convolucional ----------------
modelo = models.Sequential([
    layers.Input((28, 28, 1)),
    aumento,
    layers.Conv2D(32, 3, activation="relu"),
    layers.MaxPooling2D(),
    layers.Conv2D(64, 3, activation="relu"),
    layers.MaxPooling2D(),
    layers.Flatten(),
    layers.Dropout(0.3),
    layers.Dense(128, activation="relu"),
    layers.Dense(10, activation="softmax"),
])
modelo.summary()

modelo.compile(optimizer="adam",
               loss="sparse_categorical_crossentropy",
               metrics=["accuracy"])

modelo.fit(x_ent, y_ent, epochs=8, batch_size=128, validation_split=0.1)

perdida, exactitud = modelo.evaluate(x_pru, y_pru, verbose=0)
print(f"\nExactitud en datos de prueba: {exactitud*100:.2f} %")

modelo.save("modelo_digitos.keras")
print("Modelo guardado como modelo_digitos.keras")
