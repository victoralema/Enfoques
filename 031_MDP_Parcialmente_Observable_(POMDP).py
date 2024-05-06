#Victor Eduardo Aleman Padilla 21310193

import numpy as np

class POMDPGraphSearch:
    def __init__(self, graph, observation_model, transition_model, reward_model):
        self.graph = graph  # Grafo que representa el entorno
        self.observation_model = observation_model  # Modelo de observación (no se usa en este ejemplo)
        self.transition_model = transition_model  # Modelo de transición (no se usa en este ejemplo)
        self.reward_model = reward_model  # Modelo de recompensa (no se usa en este ejemplo)

    def search(self, start_node, horizon):
        current_node = start_node  # Nodo de inicio para la búsqueda
        total_reward = 0  # Recompensa total acumulada
        
        for t in range(horizon):  # Bucle a lo largo del horizonte de planificación
            print(f"Tiempo {t}, nodo actual: {current_node}")
            
            # Observación actual
            observation = self.observe(current_node)  # Observación del estado actual
            print(f"Observación: {observation}")
            
            # Decisión basada en la observación actual
            action = self.select_action(current_node, observation)  # Selección de la acción basada en la observación
            print(f"Acción seleccionada: {action}")
            
            # Transición al siguiente estado
            next_node = self.transition(current_node, action)  # Transición al siguiente estado
            print(f"Siguiente nodo: {next_node}")
            
            # Recompensa recibida
            reward = self.reward(current_node, action, next_node)  # Obtención de la recompensa por la transición
            total_reward += reward  # Acumulación de la recompensa
            print(f"Recompensa: {reward}")
            
            current_node = next_node  # Actualización del nodo actual para el siguiente paso
            
        print(f"Recompensa total acumulada: {total_reward}")  # Impresión de la recompensa total
    
    def observe(self, node):
        # Modelo de observación: Simplemente devuelve el nombre del nodo
        return node
    
    def select_action(self, node, observation):
        # Estrategia de selección de acción: Selecciona la primera acción disponible
        return list(self.graph[node].keys())[0]
    
    def transition(self, node, action):
        # Modelo de transición: Simplemente sigue la acción en el grafo
        return self.graph[node][action]
    
    def reward(self, node, action, next_node):
        # Modelo de recompensa: Devuelve 1 si el siguiente nodo es el objetivo, 0 de lo contrario
        if next_node == "objetivo":
            return 1
        else:
            return 0

# Definición del grafo
graph = {
    "inicio": {"mover_derecha": "A"},
    "A": {"mover_derecha": "B", "mover_abajo": "C"},
    "B": {"mover_izquierda": "A", "mover_abajo": "D"},
    "C": {"mover_arriba": "A", "mover_derecha": "D"},
    "D": {"mover_izquierda": "C", "mover_arriba": "B", "mover_derecha": "objetivo"}
}

# Creación del modelo POMDP
observation_model = None  # Modelo de observación no necesario en este ejemplo
transition_model = None  # Modelo de transición no necesario en este ejemplo
reward_model = None  # Modelo de recompensa no necesario en este ejemplo

pomdp_search = POMDPGraphSearch(graph, observation_model, transition_model, reward_model)

# Ejecución de la búsqueda
pomdp_search.search("inicio", 10)  # Iniciar la búsqueda desde el nodo de inicio
