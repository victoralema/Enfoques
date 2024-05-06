#Victor Eduardo Aleman Padilla 21310193

import tensorflow as tf  # Importa TensorFlow para el aprendizaje profundo
from tensorflow.keras import layers, models  # Importa las capas y modelos de Keras
from tensorflow.keras.datasets import mnist  # Importa el conjunto de datos MNIST
import numpy as np  # Importa NumPy para manipulación de matrices

# Cargar y preprocesar los datos
(x_train, y_train), (x_test, y_test) = mnist.load_data()  # Carga el conjunto de datos MNIST
x_train, x_test = x_train / 255.0, x_test / 255.0  # Normaliza los valores de píxeles en el rango [0, 1]

# Construir el modelo de CNN
model = models.Sequential([
    layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)),  # Primera capa convolucional con activación ReLU
    layers.MaxPooling2D((2, 2)),  # Capa de pooling para reducir el tamaño de la imagen
    layers.Conv2D(64, (3, 3), activation='relu'),  # Segunda capa convolucional con activación ReLU
    layers.MaxPooling2D((2, 2)),  # Otra capa de pooling
    layers.Conv2D(64, (3, 3), activation='relu'),  # Tercera capa convolucional con activación ReLU
    layers.Flatten(),  # Aplana la salida de las capas convolucionales para conectarlas a capas densas
    layers.Dense(64, activation='relu'),  # Capa densa con activación ReLU
    layers.Dense(10, activation='softmax')  # Capa de salida con activación Softmax para la clasificación de 10 clases
])

# Compilar el modelo
model.compile(optimizer='adam',  # Utiliza el optimizador Adam
              loss='sparse_categorical_crossentropy',  # Función de pérdida para problemas de clasificación
              metrics=['accuracy'])  # Métrica para evaluar el rendimiento del modelo

# Entrenar el modelo
model.fit(np.expand_dims(x_train, -1), y_train, epochs=5, batch_size=64, validation_split=0.1)
# Entrena el modelo utilizando los datos de entrenamiento y validación

# Evaluar el modelo
test_loss, test_acc = model.evaluate(np.expand_dims(x_test, -1), y_test)
# Evalúa el modelo utilizando los datos de prueba
print('Test accuracy:', test_acc)  # Imprime la precisión del modelo en los datos de prueba
