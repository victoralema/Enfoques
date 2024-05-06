#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importamos NumPy para trabajar con matrices y operaciones matemáticas

class MarkovChain:
    def __init__(self, transition_matrix, states):
        """
        transition_matrix: Matriz de transición que define las probabilidades de transición entre estados.
        states: Lista de estados posibles en la cadena de Markov.
        """
        self.transition_matrix = transition_matrix  # Guarda la matriz de transición en el objeto MarkovChain
        self.states = states  # Guarda la lista de estados en el objeto MarkovChain
        self.current_state = np.random.choice(states)  # Escoge un estado inicial aleatorio

    def next_state(self):
        """
        Calcula el siguiente estado basado en la matriz de transición y el estado actual.
        """
        next_state_index = np.random.choice(range(len(self.states)), p=self.transition_matrix[self.states.index(self.current_state)])  # Escoge el índice del siguiente estado basado en la matriz de transición
        self.current_state = self.states[next_state_index]  # Actualiza el estado actual

    def generate_sequence(self, steps):
        """
        Genera una secuencia de estados de longitud `steps`.
        """
        sequence = [self.current_state]  # Inicializa la secuencia con el estado actual
        for _ in range(steps):
            self.next_state()  # Calcula el siguiente estado
            sequence.append(self.current_state)  # Añade el siguiente estado a la secuencia
        return sequence  # Retorna la secuencia generada

# Definimos la matriz de transición y los estados posibles
transition_matrix = np.array([[0.7, 0.3],
                              [0.4, 0.6]])  # Matriz de transición que define las probabilidades de transición entre estados
states = ['Soleado', 'Lluvioso']  # Lista de estados posibles en la cadena de Markov

# Creamos la cadena de Markov
mc = MarkovChain(transition_matrix, states)  # Creamos una instancia de la clase MarkovChain con la matriz de transición y los estados definidos

# Generamos una secuencia de 10 estados
sequence = mc.generate_sequence(10)  # Generamos una secuencia de 10 estados usando el objeto MarkovChain
print("Secuencia generada:", sequence)  # Imprimimos la secuencia generada
