#Victor Eduardo Aleman Padilla 21310193
import numpy as np

class Perceptron:
    def __init__(self, n_input):
        # Inicializamos los pesos aleatorios y el sesgo
        self.weights = np.random.rand(n_input)  # Inicializamos los pesos para las características de entrada
        self.bias = np.random.rand()  # Inicializamos el sesgo
    
    def predict(self, inputs):
        # Calculamos la activación
        activation = np.dot(inputs, self.weights) + self.bias  # Calculamos la suma ponderada de las entradas más el sesgo
        return 1 if activation >= 0 else 0  # Devolvemos 1 si la activación es mayor o igual a 0, de lo contrario, devolvemos 0
    
    def train(self, training_inputs, labels, learning_rate=0.1, epochs=100):
        # Entrenamiento del perceptrón
        for epoch in range(epochs):
            for inputs, label in zip(training_inputs, labels):
                # Realizamos la predicción
                prediction = self.predict(inputs)
                # Actualizamos los pesos y el sesgo basados en el error
                self.weights += learning_rate * (label - prediction) * inputs  # Actualizamos los pesos
                self.bias += learning_rate * (label - prediction)  # Actualizamos el sesgo
class Adaline:
    def __init__(self, n_input):
        # Inicializamos los pesos aleatorios y el sesgo
        self.weights = np.random.rand(n_input)  # Inicializamos los pesos para las características de entrada
        self.bias = np.random.rand()  # Inicializamos el sesgo
    
    def predict(self, inputs):
        # Calculamos la activación
        activation = np.dot(inputs, self.weights) + self.bias  # Calculamos la suma ponderada de las entradas más el sesgo
        return 1 if activation >= 0 else 0  # Devolvemos 1 si la activación es mayor o igual a 0, de lo contrario, devolvemos 0
    
    def train(self, training_inputs, labels, learning_rate=0.1, epochs=100):
        # Entrenamiento de ADALINE
        for epoch in range(epochs):
            for inputs, label in zip(training_inputs, labels):
                # Calculamos la predicción
                prediction = np.dot(inputs, self.weights) + self.bias
                # Actualizamos los pesos y el sesgo basados en el error
                self.weights += learning_rate * (label - prediction) * inputs  # Actualizamos los pesos
                self.bias += learning_rate * (label - prediction)  # Actualizamos el sesgo
class Madaline:
    def __init__(self, n_input, n_output):
        # Inicializamos los pesos aleatorios y el sesgo
        self.n_input = n_input
        self.n_output = n_output
        self.weights = np.random.rand(n_input, n_output)  # Inicializamos los pesos para las características de entrada
        self.bias = np.random.rand(n_output)  # Inicializamos el sesgo
    
    def predict(self, inputs):
        # Calculamos la activación
        activation = np.dot(inputs, self.weights) + self.bias  # Calculamos la suma ponderada de las entradas más el sesgo
        return activation
    
    def train(self, training_inputs, labels, learning_rate=0.1, epochs=100):
        # Entrenamiento de MADALINE
        for epoch in range(epochs):
            for inputs, label in zip(training_inputs, labels):
                # Calculamos la predicción
                prediction = np.dot(inputs, self.weights) + self.bias
                # Actualizamos los pesos y el sesgo basados en el error
                self.weights += learning_rate * np.outer(label - prediction, inputs)  # Actualizamos los pesos
                self.bias += learning_rate * (label - prediction)  # Actualizamos el sesgo
