#Victor Eduardo Aleman Padilla 21310193
# Importamos la librería numpy para cálculos numéricos
import numpy as np

# Definimos una clase para el cálculo de probabilidades mediante etiquetado de líneas
class LineTaggingProbability:
    # Constructor de la clase que recibe una lista de resultados posibles
    def __init__(self, outcomes):
        self.outcomes = outcomes  # Almacena los resultados posibles
        self.probabilities = self.calculate_probabilities()  # Calcula las probabilidades

    # Método para calcular las probabilidades de los eventos
    def calculate_probabilities(self):
        probabilities = {}  # Diccionario para almacenar las probabilidades
        total_outcomes = len(self.outcomes)  # Calcula el total de resultados posibles
        for outcome in self.outcomes:  # Itera sobre cada resultado posible
            if outcome not in probabilities:  # Si el resultado no está en el diccionario de probabilidades
                probabilities[outcome] = 1 / total_outcomes  # Asigna una probabilidad inicial
            else:
                probabilities[outcome] += 1 / total_outcomes  # Actualiza la probabilidad si el resultado ya existe
        return probabilities  # Devuelve el diccionario de probabilidades

    # Método para obtener la probabilidad de un evento específico
    def probability(self, event):
        return self.probabilities.get(event, 0)  # Devuelve la probabilidad del evento o cero si no está presente

# Definimos todas las posibles sumas de dos dados
outcomes = [(i, j) for i in range(1, 7) for j in range(1, 7)]

# Creamos el objeto de probabilidad
prob_calc = LineTaggingProbability(outcomes)

# Calculamos la probabilidad de obtener una suma específica, por ejemplo, 7
sum_event = (3, 4)  # Cambia esto para calcular la probabilidad de otra suma
probability_of_sum = prob_calc.probability(sum_event)

# Imprimimos la probabilidad calculada
print("La probabilidad de obtener la suma", sum_event[0] + sum_event[1], "es:", probability_of_sum)
