#Victor Eduardo Aleman Padilla 21310193

import random

class MarkovChain:
    def __init__(self, transition_matrix, initial_state):#Un diccionario que representa la matriz de transición.
        
        self.transition_matrix = transition_matrix
        self.state = initial_state
    
    def next_state(self):
        """
        Calcula el siguiente estado basado en el estado actual
        y la matriz de transición.
        """
        if self.state in self.transition_matrix:
            next_states = self.transition_matrix[self.state]
            probabilities = list(next_states.values())
            next_state = random.choices(list(next_states.keys()), probabilities)[0]
            self.state = next_state
            return self.state
        else:
            raise ValueError("State not found in transition matrix.")
    
    def generate_states(self, num_steps):
        """
        Genera una secuencia de estados de acuerdo con la matriz de transición
        durante un número especificado de pasos.
        
        num_steps: int
            Número de pasos de tiempo para generar.
        """
        states = [self.state]
        for _ in range(num_steps):
            next_state = self.next_state()
            states.append(next_state)
        return states

# Definir la matriz de transición para el clima (soleado o lluvioso)
transition_matrix = {
    'Sunny': {'Sunny': 0.8, 'Rainy': 0.2},
    'Rainy': {'Sunny': 0.4, 'Rainy': 0.6}
}

# Crear una instancia de la cadena de Markov para el clima
weather_chain = MarkovChain(transition_matrix, initial_state='Sunny')

# Generar una secuencia de estados para los próximos 10 días
sequence = weather_chain.generate_states(num_steps=10)

# Imprimir la secuencia generada
print("Generated Weather Sequence:")
print(sequence)
