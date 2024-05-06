#Victor Eduardo Aleman Padilla 21310193
import numpy as np

class Perceptron:
    def __init__(self, n_input):
        # Inicializamos los pesos y el sesgo aleatorios
        self.weights = np.random.rand(n_input)
        self.bias = np.random.rand()
    
    def predict(self, inputs):
        # Calculamos la activación
        activation = np.dot(inputs, self.weights) + self.bias
        # Aplicamos la función de activación (en este caso, la función escalón)
        return 1 if activation >= 0 else 0
    
    def train(self, training_inputs, labels, learning_rate=0.1, epochs=100):
        for epoch in range(epochs):
            for inputs, label in zip(training_inputs, labels):
                # Realizamos la predicción
                prediction = self.predict(inputs)
                # Actualizamos los pesos y el sesgo basados en el error
                self.weights += learning_rate * (label - prediction) * inputs
                self.bias += learning_rate * (label - prediction)
                
# Ejemplo de uso
# Datos de entrenamiento (características) y etiquetas
training_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
labels = np.array([0, 0, 0, 1])

# Crear y entrenar el Perceptrón
perceptron = Perceptron(n_input=2)
perceptron.train(training_inputs, labels)

# Hacer algunas predicciones
test_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
for inputs in test_inputs:
    print(f"Input: {inputs}, Prediction: {perceptron.predict(inputs)}")
