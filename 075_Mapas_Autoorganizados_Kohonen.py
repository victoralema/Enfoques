#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importamos la biblioteca NumPy para trabajar con matrices y operaciones numéricas
import matplotlib.pyplot as plt  # Importamos matplotlib para visualización

class SOM:
    def __init__(self, input_size, output_size, learning_rate=0.1, sigma=1.0):
        # Inicializamos la clase SOM con los tamaños de entrada y salida, tasa de aprendizaje y sigma (radio de influencia)
        self.input_size = input_size  # Tamaño de la entrada
        self.output_size = output_size  # Tamaño de la salida (número de neuronas en el mapa)
        self.weights = np.random.rand(output_size, input_size)  # Inicializamos los pesos aleatoriamente
        self.learning_rate = learning_rate  # Tasa de aprendizaje
        self.sigma = sigma  # Sigma (radio de influencia)

    def train(self, data, epochs):
        # Entrenamos el SOM con los datos de entrada
        for epoch in range(epochs):
            for d in data:  # Iteramos sobre los datos
                bmu_index = self.find_bmu(d)  # Encontramos la Mejor Unidad de Coincidencia (BMU)
                self.update_weights(d, bmu_index, epoch, epochs)  # Actualizamos los pesos basados en la BMU

    def find_bmu(self, input_vector):
        # Encontramos la Mejor Unidad de Coincidencia (BMU) para un vector de entrada
        distances = np.linalg.norm(self.weights - input_vector, axis=1)  # Calculamos las distancias entre los pesos y el vector de entrada
        return np.argmin(distances)  # Devolvemos el índice de la BMU

    def update_weights(self, input_vector, bmu_index, epoch, max_epochs):
        # Actualizamos los pesos basados en el vector de entrada y la BMU
        influence = self.get_influence(epoch, max_epochs)  # Calculamos la influencia basada en la época actual
        for i, weight in enumerate(self.weights):  # Iteramos sobre los pesos
            distance_to_bmu = np.abs(bmu_index - i)  # Calculamos la distancia a la BMU
            if distance_to_bmu <= influence:  # Si la distancia está dentro del radio de influencia
                self.weights[i] += self.learning_rate * influence * (input_vector - weight)  # Actualizamos el peso

    def get_influence(self, epoch, max_epochs):
        # Calculamos la influencia basada en la época actual
        initial_sigma = self.sigma  # Sigma inicial
        return initial_sigma * (1 - epoch / max_epochs)  # Calculamos la influencia

    def predict(self, data):
        # Realizamos predicciones para los datos de entrada
        predictions = []
        for d in data:  # Iteramos sobre los datos
            bmu_index = self.find_bmu(d)  # Encontramos la BMU para cada dato
            predictions.append(bmu_index)  # Agregamos la BMU a las predicciones
        return predictions

# Datos de ejemplo (2 dimensiones)
data = np.array([[0.2, 0.5],
                 [0.6, 0.8],
                 [0.8, 0.2],
                 [0.3, 0.9],
                 [0.7, 0.3]])

# Normalizar los datos
data = (data - data.min()) / (data.max() - data.min())

# Definir y entrenar el SOM
som = SOM(input_size=2, output_size=2, learning_rate=0.1, sigma=1.0)
som.train(data, epochs=100)

# Visualización
plt.scatter(data[:,0], data[:,1])  # Scatter plot de los datos
for i, w in enumerate(som.weights):  # Iteramos sobre los pesos del SOM
    plt.text(w[0], w[1], str(i), fontsize=12, ha='center', va='center')  # Añadimos etiquetas a los puntos
plt.title('Mapa Autoorganizado de Kohonen')  # Título del gráfico
plt.xlabel('Feature 1')  # Etiqueta del eje x
plt.ylabel('Feature 2')  # Etiqueta del eje y
plt.show()  # Mostramos el gráfico
