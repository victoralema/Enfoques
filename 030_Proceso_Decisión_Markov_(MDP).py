#Victor Eduardo Aleman Padilla 21310193

import numpy as np

# Definición de la clase para los estados del MDP
class State:
    def __init__(self, name):
        self.name = name  # Nombre del estado
        self.transitions = {}  # Diccionario de transiciones a otros estados

# Algoritmo de Iteración de Valor para MDP
def value_iteration_mdp(states, gamma=0.9, threshold=0.01):
    # Inicialización de los valores de los estados
    values = {state: 0 for state in states}
    
    while True:
        delta = 0
        # Iterar sobre cada estado en el MDP
        for state in states:
            old_value = values[state]
            # Calcular el nuevo valor del estado
            if state.transitions:
                max_value = max([sum([transition[1] * values[transition[0]] for transition in state.transitions[action]]) for action in state.transitions])
                values[state] = max_value
            # Calcular la diferencia entre el valor nuevo y el antiguo
            delta = max(delta, abs(old_value - values[state]))
        # Verificar si se ha alcanzado la convergencia
        if delta < threshold:
            break
    
    # Devolver los valores óptimos de los estados
    return values

# Ejemplo de uso
if __name__ == "__main__":
    # Definir los estados del MDP
    s0 = State("S0")
    s1 = State("S1")
    s2 = State("S2")
    s3 = State("S3")
    # Definir las transiciones entre estados (estado, probabilidad, recompensa)
    s0.transitions = {'a': [(s1, 0.2, 5), (s2, 0.8, 2)],
                      'b': [(s2, 1.0, 1)]}
    s1.transitions = {'a': [(s0, 0.6, -1), (s2, 0.4, 3)]}
    s2.transitions = {'a': [(s3, 0.5, 0), (s0, 0.5, 4)],
                      'b': [(s3, 1.0, -1)]}
    s3.transitions = {'a': [(s3, 1.0, 10)]}
    states = [s0, s1, s2, s3]
    # Ejecutar el algoritmo de Iteración de Valor para MDP
    optimal_values = value_iteration_mdp(states)
    print("Valores óptimos de los estados:")
    for state, value in optimal_values.items():
        print(f"Estado: {state.name}, Valor óptimo: {value}")
