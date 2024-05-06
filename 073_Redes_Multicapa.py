#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importamos NumPy para operaciones numéricas
import tensorflow as tf  # Importamos TensorFlow para construir y entrenar redes neuronales
from sklearn.model_selection import train_test_split  # Importamos train_test_split para dividir los datos en conjuntos de entrenamiento y prueba

# Datos de ejemplo: características y etiquetas
X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])  # Definimos las características de entrada
y = np.array([[0], [1], [1], [0]])  # Definimos las etiquetas de salida

# Dividir los datos en conjuntos de entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)  # Dividimos los datos con un 80% para entrenamiento y un 20% para prueba

# Definir el modelo de red neuronal
model = tf.keras.Sequential([  # Creamos un modelo secuencial
    tf.keras.layers.Dense(2, input_shape=(2,), activation='relu'),  # Capa oculta con 2 neuronas y función de activación ReLU
    tf.keras.layers.Dense(1, activation='sigmoid')  # Capa de salida con 1 neurona y función de activación sigmoide
])

# Compilar el modelo
model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])  # Compilamos el modelo con el optimizador Adam y la pérdida de entropía cruzada binaria

# Entrenar el modelo
model.fit(X_train, y_train, epochs=100, verbose=1)  # Entrenamos el modelo durante 100 épocas

# Evaluar el modelo en el conjunto de prueba
loss, accuracy = model.evaluate(X_test, y_test)  # Evaluamos el modelo en el conjunto de prueba para obtener la pérdida y la precisión
print(f'Loss: {loss}, Accuracy: {accuracy}')  # Imprimimos la pérdida y la precisión del modelo en el conjunto de prueba

# Hacer predicciones
predictions = model.predict(X_test)  # Realizamos predicciones en el conjunto de prueba
print("Predictions:")  # Imprimimos un mensaje indicando las predicciones
print(predictions)  # Imprimimos las predicciones realizadas por el modelo
