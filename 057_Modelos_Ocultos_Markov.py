#Victor Eduardo Aleman Padilla 21310193

class HMM:
    def __init__(self):
        # Probabilidades iniciales de los estados ocultos
        self.initial_probabilities = {'S1': 0.6, 'S2': 0.3, 'S3': 0.1}

        # Matriz de transición de los estados ocultos
        self.transition_matrix = {
            'S1': {'S1': 0.7, 'S2': 0.2, 'S3': 0.1},
            'S2': {'S1': 0.3, 'S2': 0.5, 'S3': 0.2},
            'S3': {'S1': 0.1, 'S2': 0.4, 'S3': 0.5}
        }

        # Matriz de emisión de los símbolos observados
        self.emission_matrix = {
            'S1': {'O1': 0.3, 'O2': 0.4, 'O3': 0.3},
            'S2': {'O1': 0.4, 'O2': 0.3, 'O3': 0.3},
            'S3': {'O1': 0.2, 'O2': 0.3, 'O3': 0.5}
        }

    def forward_algorithm(self, observations):
        # Inicialización
        alpha = {}
        for state in self.initial_probabilities:
            # Calcula el producto de la probabilidad inicial del estado y la probabilidad de emisión de la primera observación para ese estado
            alpha[state] = self.initial_probabilities[state] * self.emission_matrix[state][observations[0]]

        # Paso hacia adelante (forward)
        for t in range(1, len(observations)):
            new_alpha = {}
            for state_to in self.transition_matrix:
                # Calcula la suma ponderada de las probabilidades hacia adelante anteriores multiplicadas por las probabilidades de transición
                total = sum(alpha[state_from] * self.transition_matrix[state_from][state_to] 
                            for state_from in self.transition_matrix)
                # Multiplica la suma ponderada por la probabilidad de emisión de la observación actual para el estado actual
                new_alpha[state_to] = total * self.emission_matrix[state_to][observations[t]]
            alpha = new_alpha

        return alpha

    def viterbi_algorithm(self, observations):
        # Inicialización
        delta = {}
        psi = {}
        for state in self.initial_probabilities:
            # Inicializa delta con el producto de la probabilidad inicial del estado y la probabilidad de emisión de la primera observación para ese estado
            delta[state] = self.initial_probabilities[state] * self.emission_matrix[state][observations[0]]
            # Inicializa psi con 0 para cada estado
            psi[state] = 0

        # Paso hacia adelante (forward)
        for t in range(1, len(observations)):
            new_delta = {}
            new_psi = {}
            for state_to in self.transition_matrix:
                # Calcula el máximo de delta anterior multiplicado por la probabilidad de transición
                max_prob = max(delta[state_from] * self.transition_matrix[state_from][state_to] 
                               for state_from in self.transition_matrix)
                # Actualiza delta con el máximo encontrado multiplicado por la probabilidad de emisión de la observación actual para el estado actual
                new_delta[state_to] = max_prob * self.emission_matrix[state_to][observations[t]]
                # Encuentra el estado anterior que maximiza la probabilidad
                argmax_state = max(self.transition_matrix, key=lambda state_from: delta[state_from] * self.transition_matrix[state_from][state_to])
                # Actualiza psi con el estado anterior que maximiza la probabilidad
                new_psi[state_to] = argmax_state
            delta = new_delta
            psi = new_psi

        # Reconstrucción de la secuencia de estados ocultos
        sequence = [max(delta, key=delta.get)]
        for t in range(len(observations) - 1, 0, -1):
            # Añade el estado anterior que maximiza la probabilidad al principio de la secuencia
            sequence.insert(0, psi[sequence[0]])

        return sequence


# Ejemplo de uso
hmm = HMM()
observations = ['O1', 'O2', 'O3']  # Secuencia observada
alpha = hmm.forward_algorithm(observations)
print("Probabilidad posterior de los estados ocultos (forward):", alpha)
most_probable_sequence = hmm.viterbi_algorithm(observations)
print("Secuencia más probable de estados ocultos (Viterbi):", most_probable_sequence)
