#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importamos la biblioteca NumPy para realizar cálculos numéricos

class NeuralNetwork:
    def __init__(self, input_size, hidden_size, output_size, learning_rate):
        # Inicialización de los parámetros de la red neuronal
        self.input_size = input_size  # Número de neuronas en la capa de entrada
        self.hidden_size = hidden_size  # Número de neuronas en la capa oculta
        self.output_size = output_size  # Número de neuronas en la capa de salida
        self.learning_rate = learning_rate  # Tasa de aprendizaje
        
        # Inicialización de los pesos y sesgos de manera aleatoria
        self.weights_input_hidden = np.random.randn(self.input_size, self.hidden_size)  # Pesos entre la capa de entrada y la capa oculta
        self.bias_input_hidden = np.random.randn(self.hidden_size)  # Sesgos de la capa oculta
        self.weights_hidden_output = np.random.randn(self.hidden_size, self.output_size)  # Pesos entre la capa oculta y la capa de salida
        self.bias_hidden_output = np.random.randn(self.output_size)  # Sesgos de la capa de salida
        
    def sigmoid(self, x):
        # Función de activación sigmoide
        return 1 / (1 + np.exp(-x))
    
    def sigmoid_derivative(self, x):
        # Derivada de la función de activación sigmoide
        return x * (1 - x)
    
    def forward(self, X):
        # Propagación hacia adelante
        self.hidden_output = self.sigmoid(np.dot(X, self.weights_input_hidden) + self.bias_input_hidden)  # Salida de la capa oculta
        self.predictions = self.sigmoid(np.dot(self.hidden_output, self.weights_hidden_output) + self.bias_hidden_output)  # Predicciones de la red neuronal
    
    def backward(self, X, y):
        # Retropropagación del error
        error = y - self.predictions  # Error entre las predicciones y las etiquetas reales
        output_delta = error * self.sigmoid_derivative(self.predictions)  # Cálculo del delta de la capa de salida
        hidden_error = np.dot(output_delta, self.weights_hidden_output.T)  # Error en la capa oculta
        hidden_delta = hidden_error * self.sigmoid_derivative(self.hidden_output)  # Cálculo del delta de la capa oculta
        
        # Actualización de los pesos y sesgos utilizando el gradiente descendente
        self.weights_hidden_output += np.dot(self.hidden_output.T, output_delta) * self.learning_rate  # Actualización de los pesos entre la capa oculta y la capa de salida
        self.bias_hidden_output += np.sum(output_delta) * self.learning_rate  # Actualización de los sesgos de la capa de salida
        self.weights_input_hidden += np.dot(X.T, hidden_delta) * self.learning_rate  # Actualización de los pesos entre la capa de entrada y la capa oculta
        self.bias_input_hidden += np.sum(hidden_delta) * self.learning_rate  # Actualización de los sesgos de la capa oculta
        
    def train(self, X, y, epochs):
        # Entrenamiento de la red neuronal
        for epoch in range(epochs):
            self.forward(X)  # Propagación hacia adelante
            self.backward(X, y)  # Retropropagación del error
            
            # Imprimir la pérdida cada 1000 épocas
            if epoch % 1000 == 0:
                print(f'Epoch {epoch}, Loss: {np.mean(np.abs(y - self.predictions))}')  # Calculamos y mostramos la pérdida

# Datos de entrenamiento y etiquetas
X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])  # Datos de entrada
y = np.array([[0], [1], [1], [0]])  # Etiquetas

# Crear una instancia de la red neuronal
nn = NeuralNetwork(input_size=2, hidden_size=4, output_size=1, learning_rate=0.1)

# Entrenar la red neuronal
nn.train(X, y, epochs=10000)  # Entrenamos la red neuronal durante 10000 épocas
