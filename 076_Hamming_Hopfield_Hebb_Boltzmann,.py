#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importamos la biblioteca NumPy para trabajar con matrices y operaciones numéricas

### Modelo de Hopfield ###

class HopfieldNetwork:
    def __init__(self, size):
        # Inicializamos la red de Hopfield con un tamaño dado
        self.size = size  # Tamaño de la red (número de neuronas)
        self.weights = np.zeros((size, size))  # Inicializamos la matriz de pesos como una matriz de ceros

    def train(self, patterns):
        # Entrenamos la red de Hopfield con un conjunto de patrones dados
        for pattern in patterns:  # Iteramos sobre cada patrón en el conjunto de patrones
            self.weights += np.outer(pattern, pattern)  # Actualizamos los pesos basados en el producto externo de cada patrón consigo mismo

        np.fill_diagonal(self.weights, 0)  # Rellenamos la diagonal de la matriz de pesos con ceros para evitar que las neuronas se conecten a sí mismas

    def predict(self, patterns, steps=5):
        # Realizamos predicciones utilizando la red de Hopfield
        predicted_patterns = []  # Inicializamos una lista para almacenar las predicciones
        for pattern in patterns:  # Iteramos sobre cada patrón en el conjunto de patrones
            state = np.copy(pattern)  # Inicializamos el estado de la red con el patrón de entrada
            for _ in range(steps):  # Iteramos un número determinado de veces (pasos)
                state = np.sign(np.dot(self.weights, state))  # Actualizamos el estado de la red aplicando la función de activación signo al producto punto de los pesos y el estado actual
            predicted_patterns.append(state)  # Agregamos la predicción a la lista de predicciones
        return predicted_patterns  # Devolvemos todas las predicciones realizadas por la red de Hopfield

### Modelo de Boltzmann ###

class BoltzmannMachine:
    def __init__(self, size):
        # Inicializamos la máquina de Boltzmann con un tamaño dado
        self.size = size  # Tamaño de la máquina (número de neuronas)
        self.weights = np.random.randn(size, size)  # Inicializamos los pesos de manera aleatoria utilizando una distribución normal

    def train(self, patterns, learning_rate=0.1, epochs=100):
        # Entrenamos la máquina de Boltzmann con un conjunto de patrones dados durante un número determinado de épocas
        for _ in range(epochs):  # Iteramos sobre el número de épocas
            for pattern in patterns:  # Iteramos sobre cada patrón en el conjunto de patrones
                self.update_weights(pattern, learning_rate)  # Actualizamos los pesos basados en el patrón actual y la tasa de aprendizaje

    def update_weights(self, pattern, learning_rate):
        # Actualizamos los pesos de la máquina de Boltzmann basados en un patrón de entrada y una tasa de aprendizaje dada
        for i in range(self.size):  # Iteramos sobre las neuronas en la máquina
            for j in range(self.size):  # Iteramos sobre las conexiones entre las neuronas
                if i != j:  # Evitamos que las neuronas se conecten a sí mismas
                    self.weights[i, j] += learning_rate * pattern[i] * pattern[j]  # Actualizamos el peso entre las neuronas i y j basado en el producto de las activaciones de las neuronas correspondientes en el patrón de entrada

    def predict(self, patterns, steps=5):
        # Realizamos predicciones utilizando la máquina de Boltzmann
        predicted_patterns = []  # Inicializamos una lista para almacenar las predicciones
        for pattern in patterns:  # Iteramos sobre cada patrón en el conjunto de patrones
            state = np.copy(pattern)  # Inicializamos el estado de la máquina con el patrón de entrada
            for _ in range(steps):  # Iteramos un número determinado de veces (pasos)
                state = np.sign(np.dot(self.weights, state))  # Actualizamos el estado de la máquina aplicando la función de activación signo al producto punto de los pesos y el estado actual
            predicted_patterns.append(state)  # Agregamos la predicción a la lista de predicciones
        return predicted_patterns  # Devolvemos todas las predicciones realizadas por la máquina de Boltzmann

### Ejemplo de uso ###

# Datos de ejemplo
patterns = [[1, -1, 1, -1],
            [-1, -1, -1, 1],
            [1, 1, -1, -1]]

# Crear y entrenar la red de Hopfield
hopfield_net = HopfieldNetwork(size=len(patterns[0]))
hopfield_net.train(patterns)

# Crear y entrenar la máquina de Boltzmann
boltzmann_machine = BoltzmannMachine(size=len(patterns[0]))
boltzmann_machine.train(patterns)

# Probar los modelos con los datos de ejemplo
print("Patrones de entrada:")
for pattern in patterns:
    print(pattern)

print("\nPredicciones del modelo de Hopfield:")
print(hopfield_net.predict(patterns))

print("\nPredicciones de la máquina de Boltzmann:")
print(boltzmann_machine.predict(patterns))
