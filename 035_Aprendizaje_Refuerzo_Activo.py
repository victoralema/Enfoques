#Victor Eduardo Aleman Padilla 21310193

import networkx as nx  # Importamos la biblioteca networkx para trabajar con grafos
import random  # Importamos la biblioteca random para generación de números aleatorios

# Crear un grafo simple usando networkx
G = nx.Graph()  # Creamos un grafo no dirigido vacío
edges = [(0, 1), (0, 2), (1, 3), (2, 3)]  # Definimos las aristas del grafo
G.add_edges_from(edges)  # Agregamos las aristas al grafo

# Definir el estado inicial y el estado objetivo
start_node = 0  # Nodo inicial del agente
goal_node = 3  # Nodo objetivo al que el agente debe llegar

# Inicializar la matriz Q con valores arbitrarios
num_nodes = len(G.nodes)  # Obtenemos el número de nodos en el grafo
Q = [[0] * num_nodes for _ in range(num_nodes)]  # Creamos una matriz Q inicializada con ceros

# Hiperparámetros del algoritmo Q-learning
learning_rate = 0.5  # Tasa de aprendizaje del agente
discount_factor = 0.8  # Factor de descuento para futuras recompensas
epsilon = 0.2  # Factor de exploración (epsilon-greedy)
num_episodes = 1000  # Número de episodios de aprendizaje

# Función para seleccionar la próxima acción basada en la política epsilon-greedy
def choose_next_action(state):
    if random.uniform(0, 1) < epsilon:
        # Exploración: elige aleatoriamente una acción no explorada
        neighbors = list(G.neighbors(state))
        unexplored_neighbors = [n for n in neighbors if Q[state][n] == 0]  # Filtramos acciones no exploradas
        if unexplored_neighbors:
            return random.choice(unexplored_neighbors)  # Si hay acciones no exploradas, elige una al azar
        else:
            return random.choice(neighbors)  # Si no hay acciones no exploradas, elige una al azar de todas
    else:
        # Explotación: elige la acción con el valor Q más alto
        neighbors = list(G.neighbors(state))
        return max(neighbors, key=lambda n: Q[state][n])  # Elige la acción con el mayor valor Q

# Algoritmo Q-learning
for _ in range(num_episodes):
    current_state = start_node  # Inicializamos el estado actual como el nodo inicial
    while current_state != goal_node:
        next_action = choose_next_action(current_state)  # Seleccionamos la próxima acción
        reward = -1 if next_action != goal_node else 0  # Asignamos la recompensa (penalización en cada paso, 0 al llegar al objetivo)
        
        # Actualizar la matriz Q usando la ecuación de Q-learning
        best_next_action = max(list(G.neighbors(next_action)), key=lambda n: Q[next_action][n])
        Q[current_state][next_action] += learning_rate * (reward + discount_factor * Q[next_action][best_next_action] - Q[current_state][next_action])
        
        # Mover al siguiente estado
        current_state = next_action  # Actualizamos el estado actual al siguiente estado (acción elegida)

# Encontrar el camino óptimo desde el nodo inicial al nodo objetivo
current_node = start_node  # Comenzamos desde el nodo inicial
optimal_path = [current_node]  # Inicializamos la lista de camino óptimo con el nodo inicial

while current_node != goal_node:
    next_node = max(list(G.neighbors(current_node)), key=lambda n: Q[current_node][n])  # Elegimos el siguiente nodo basado en el mayor valor Q
    optimal_path.append(next_node)  # Agregamos el siguiente nodo al camino óptimo
    current_node = next_node  # Actualizamos el nodo actual al siguiente nodo

print("Camino óptimo encontrado por el agente:", optimal_path)  # Imprimimos el camino óptimo encontrado
