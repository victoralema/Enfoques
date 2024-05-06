#Victor Eduardo Aleman Padilla 21310193
# Importamos el módulo random para generar números aleatorios
import random

# Definimos una clase para el cálculo de probabilidades mediante movimientos
class ProbabilityMovement:
    # Constructor de la clase que recibe el número de lados del dado (por defecto 6)
    def __init__(self, num_sides=6):
        self.num_sides = num_sides  # Almacenamos el número de lados del dado

    # Método para simular el lanzamiento de un dado
    def roll_dice(self):
        return random.randint(1, self.num_sides)  # Generamos un número aleatorio entre 1 y el número de lados

    # Método para calcular la probabilidad de obtener un número específico
    def calculate_probability(self, target_num, num_trials=10000):
        num_successes = 0  # Contador para el número de éxitos
        for _ in range(num_trials):  # Realizamos un número determinado de ensayos
            result = self.roll_dice()  # Lanzamos el dado
            if result == target_num:  # Si el resultado coincide con el número objetivo
                num_successes += 1  # Incrementamos el contador de éxitos
        probability = num_successes / num_trials  # Calculamos la probabilidad como éxitos dividido entre ensayos totales
        return probability  # Devolvemos la probabilidad calculada

# Creamos un objeto para el cálculo de probabilidad
prob_calc = ProbabilityMovement()

# Definimos el número objetivo y calculamos su probabilidad
target_number = 4  # Número que queremos calcular su probabilidad
probability_of_target = prob_calc.calculate_probability(target_number)

# Imprimimos el resultado
print("La probabilidad de obtener el número", target_number, "es:", probability_of_target)
