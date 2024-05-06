#Victor Eduardo Aleman Padilla 21310193

import numpy as np

class DynamicBayesianNetwork:
    def __init__(self, graph, transition_model, observation_model):
        self.graph = graph  # Estructura del grafo que representa las relaciones entre las variables
        self.transition_model = transition_model  # Modelo de transición que define cómo evolucionan las variables en el tiempo
        self.observation_model = observation_model  # Modelo de observación que mapea las observaciones a las variables
    
    def inference(self, observations, time_steps):
        belief_state = np.zeros((len(self.graph), time_steps))  # Estado de creencia inicial
        
        # Iterar sobre cada paso de tiempo
        for t in range(time_steps):
            if t == 0:
                # Inicialización: Utiliza las observaciones iniciales para actualizar el estado de creencia
                belief_state[:, t] = self.update_belief_initial(observations[:, t])
            else:
                # Predicción: Utiliza el modelo de transición para predecir el siguiente estado
                predicted_belief = self.transition_model @ belief_state[:, t - 1]
                
                # Actualización: Incorpora las nuevas observaciones utilizando el modelo de observación
                belief_state[:, t] = self.update_belief(predicted_belief, observations[:, t])
        
        return belief_state
    
    def update_belief_initial(self, initial_observations):
        # Utiliza las observaciones iniciales para calcular el estado de creencia inicial
        belief_state_initial = np.zeros(len(self.graph))
        for i, obs in enumerate(initial_observations):
            belief_state_initial[i] = self.observation_model[i][obs]
        return belief_state_initial
    
    def update_belief(self, predicted_belief, new_observations):
        # Incorpora las nuevas observaciones al estado de creencia predicho
        updated_belief = predicted_belief.copy()
        for i, obs in enumerate(new_observations):
            updated_belief[i] *= self.observation_model[i][obs]
        # Normaliza el estado de creencia para que la suma sea 1
        updated_belief /= np.sum(updated_belief)
        return updated_belief

# Definición del grafo (representado como una matriz de adyacencia)
graph = np.array([[0, 1, 0],
                  [0, 0, 1],
                  [0, 0, 0]])

# Modelo de transición (probabilidades de transición entre estados)
transition_model = np.array([[0.7, 0.3, 0],
                             [0, 0.6, 0.4],
                             [0, 0, 1.0]])

# Modelo de observación (probabilidades de observación para cada estado)
observation_model = [{0: 0.8, 1: 0.2},  # Variable 1
                     {0: 0.4, 1: 0.6},  # Variable 2
                     {0: 0.9, 1: 0.1}]  # Variable 3

# Creación de la red bayesiana dinámica
dbn = DynamicBayesianNetwork(graph, transition_model, observation_model)

# Observaciones a lo largo del tiempo
observations = np.array([[1, 0, 1],
                         [0, 1, 1],
                         [1, 1, 0]])

# Número de pasos de tiempo
time_steps = observations.shape[1]

# Inferencia utilizando la red bayesiana dinámica
belief_state = dbn.inference(observations, time_steps)

# Imprimir el estado de creencia final
print("Estado de creencia final:")
print(belief_state[:, -1])
