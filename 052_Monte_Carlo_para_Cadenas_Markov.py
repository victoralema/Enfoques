#Victor Eduardo Aleman Padilla 21310193

import random

class MarkovChain:
    def __init__(self, transition_matrix, states):
        """
        Initialize the Markov Chain with a transition matrix and state space.
        
        Args:
        transition_matrix (list of lists): The transition probabilities between states.
        states (list): The list of possible states in the chain.
        """
        self.transition_matrix = transition_matrix
        self.states = states
        self.state = random.choice(states)  # Start with a random initial state

    def next_state(self):
        """
        Move to the next state based on the current state and transition probabilities.
        
        Returns:
        str: The next state.
        """
        transition_probabilities = self.transition_matrix[self.states.index(self.state)]
        next_state_index = random.choices(range(len(self.states)), weights=transition_probabilities)[0]
        self.state = self.states[next_state_index]
        return self.state

    def generate_sequence(self, num_steps):
        """
        Generate a sequence of states by simulating the Markov Chain.
        
        Args:
        num_steps (int): The number of steps in the Markov Chain simulation.
        
        Returns:
        list: The sequence of states visited during the simulation.
        """
        sequence = [self.state]
        for _ in range(num_steps - 1):
            next_state = self.next_state()
            sequence.append(next_state)
        return sequence


# Ejemplo de uso
transition_matrix = [
    [0.7, 0.3],  # Probabilidades de transición desde el estado 0
    [0.4, 0.6]   # Probabilidades de transición desde el estado 1
]

states = ['Estado 0', 'Estado 1']

# Crear la cadena de Markov
mc = MarkovChain(transition_matrix, states)

# Generar una secuencia de estados de longitud 10
sequence = mc.generate_sequence(10)
print("Secuencia de estados generada por la cadena de Markov:")
print(sequence)
