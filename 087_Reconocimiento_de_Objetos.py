#Victor Eduardo Aleman Padilla 21310193
import numpy as np

class NaiveBayesClassifier:
    def __init__(self):
        self.classes = []  # Lista para almacenar las clases únicas
        self.class_probabilities = {}  # Diccionario para almacenar las probabilidades de cada clase
        self.feature_probabilities = {}  # Diccionario para almacenar las probabilidades de las características

    def train(self, X, y):
        # Obtener las clases únicas en los datos
        self.classes = np.unique(y)

        # Calcular la probabilidad de cada clase
        total_samples = len(y)  # Número total de muestras
        for c in self.classes:
            # Calcular la probabilidad de cada clase
            class_samples = np.sum(y == c)  # Número de muestras en la clase actual
            self.class_probabilities[c] = class_samples / total_samples  # Probabilidad de la clase

            # Calcular las probabilidades de las características para cada clase
            class_data = X[y == c]  # Datos correspondientes a la clase actual
            self.feature_probabilities[c] = {
                "mean": np.mean(class_data, axis=0),  # Media de las características
                "std": np.std(class_data, axis=0)  # Desviación estándar de las características
            }

    def predict(self, X):
        predictions = []  # Lista para almacenar las predicciones
        for sample in X:
            # Calcular la probabilidad de cada clase para la muestra dada
            class_scores = []  # Lista para almacenar los puntajes de clase
            for c in self.classes:
                class_probability = self.class_probabilities[c]  # Probabilidad de la clase
                feature_probabilities = self.feature_probabilities[c]  # Probabilidades de las características

                # Calcular la probabilidad utilizando el teorema de Bayes
                log_prob = np.sum(
                    -0.5 * np.log(2 * np.pi * feature_probabilities['std'] ** 2) - \
                    ((sample - feature_probabilities['mean']) ** 2) / (2 * feature_probabilities['std'] ** 2)
                )  # Probabilidad logarítmica de la muestra dada la clase actual
                class_scores.append(np.log(class_probability) + log_prob)  # Puntuación de la clase

            # Seleccionar la clase con la mayor probabilidad
            predicted_class = self.classes[np.argmax(class_scores)]  # Clase con el mayor puntaje
            predictions.append(predicted_class)  # Agregar la predicción a la lista

        return predictions

# Ejemplo de uso
if __name__ == "__main__":
    # Datos de ejemplo (características de las imágenes)
    X_train = np.array([[5, 1], [2, 3], [3, 4], [4, 2], [1, 5], [4, 4]])
    y_train = np.array(['gato', 'perro', 'perro', 'gato', 'gato', 'perro'])

    # Inicializar y entrenar el clasificador
    classifier = NaiveBayesClassifier()
    classifier.train(X_train, y_train)

    # Datos de prueba
    X_test = np.array([[3, 2], [1, 4], [4, 3]])

    # Realizar predicciones
    predictions = classifier.predict(X_test)
    print("Predicciones:", predictions)
