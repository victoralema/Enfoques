#Victor Eduardo Aleman Padilla 21310193

import numpy as np

class RedNeuronal:
    def __init__(self):
        # Inicializamos los pesos y sesgos aleatoriamente
        self.w = np.random.randn(2)  # Inicializamos los pesos aleatorios para dos características
        self.b = np.random.randn(1)  # Inicializamos el sesgo aleatorio
        
    def sigmoide(self, x):
        return 1 / (1 + np.exp(-x))  # Definimos la función sigmoide
    
    def relu(self, x):
        return np.maximum(0, x)  # Definimos la función ReLU
    
    def predict(self, x):
        # Calculamos la salida de la red neuronal utilizando la función sigmoide
        return self.sigmoide(np.dot(x, self.w) + self.b)  # Aplicamos la función sigmoide a la suma ponderada de las entradas más el sesgo
red_neuronal = RedNeuronal()  # Creamos una instancia de la clase RedNeuronal

# Datos de entrada (horas de estudio y horas de sueño)
horas_estudio = 5
horas_sueño = 7

# Normalizamos los datos de entrada (opcional)
datos_entrada = np.array([horas_estudio, horas_sueño]) / 10.0  # Normalizamos las características para que estén en el rango [0, 1]

# Realizamos la predicción
probabilidad_aprobar = red_neuronal.predict(datos_entrada)  # Utilizamos la red neuronal para predecir la probabilidad de aprobar

print("Probabilidad de aprobar el examen:", probabilidad_aprobar)  # Imprimimos la probabilidad predicha
