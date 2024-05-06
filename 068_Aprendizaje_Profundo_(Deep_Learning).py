#Victor Eduardo Aleman Padilla 21310193


import numpy as np  # Importa la biblioteca NumPy para manipulación de matrices
import tensorflow as tf  # Importa TensorFlow para construir y entrenar modelos de aprendizaje profundo

# Generar datos de entrenamiento
X_train = np.random.rand(1000, 10)  # Genera 1000 muestras con 10 características aleatorias
Y_train = np.random.randint(2, size=(1000, 1))  # Genera 1000 etiquetas binarias aleatorias

# Definir el modelo de red neuronal
model = tf.keras.Sequential([  # Crea un modelo secuencial donde las capas se apilan una encima de la otra
    tf.keras.Input(shape=(10,)),  # Capa de entrada con 10 nodos (características)
    tf.keras.layers.Dense(64, activation='relu'),  # Capa densa con 64 nodos y activación ReLU
    tf.keras.layers.Dense(32, activation='relu'),  # Capa densa con 32 nodos y activación ReLU
    tf.keras.layers.Dense(1, activation='sigmoid')  # Capa de salida con 1 nodo y activación sigmoide
])

# Compilar el modelo
model.compile(optimizer='adam',  # Optimizador Adam para optimización
              loss='binary_crossentropy',  # Función de pérdida de entropía cruzada binaria
              metrics=['accuracy'])  # Métrica para evaluar el rendimiento del modelo

# Entrenar el modelo
model.fit(X_train, Y_train, epochs=10, batch_size=32)  # Entrena el modelo durante 10 épocas con un tamaño de lote de 32

# Generar datos de prueba
X_test = np.random.rand(100, 10)  # Genera 100 muestras de prueba con 10 características aleatorias
Y_test = np.random.randint(2, size=(100, 1))  # Genera 100 etiquetas binarias de prueba aleatorias

# Evaluar el modelo
loss, accuracy = model.evaluate(X_test, Y_test)  # Evalúa el modelo en los datos de prueba
print("Loss:", loss)  # Imprime la pérdida obtenida en los datos de prueba
print("Accuracy:", accuracy)  # Imprime la precisión obtenida en los datos de prueba
