#Victor Eduardo Aleman Padilla 21310193
import random

# Definición del grafo (representado como un diccionario de listas de adyacencia)
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'F'],
    'D': ['B'],
    'E': ['B', 'F'],
    'F': ['C', 'E']
}

# Inicialización de la matriz de visitas y recompensas
num_nodes = len(graph)
visit_count = {node: 0 for node in graph}
reward_matrix = {node: {neighbor: 0 for neighbor in neighbors} for node, neighbors in graph.items()}

# Función de búsqueda en anchura (BFS)
def bfs_search(start, goal):
    queue = [[start]]  # Cola de nodos por explorar, empezando desde el nodo inicial

    while queue:
        path = queue.pop(0)
        current_node = path[-1]

        if current_node == goal:
            return path  # Se encontró el camino hacia el nodo objetivo

        for neighbor in graph[current_node]:
            new_path = list(path)
            new_path.append(neighbor)
            queue.append(new_path)

    return None  # No se encontró ningún camino hacia el nodo objetivo

# Función para realizar la exploración utilizando BFS y actualizar la matriz de recompensas
def explore_and_update(start_node):
    goal_node = random.choice(list(graph.keys()))
    
    if start_node == goal_node:
        return  # Evitar seleccionar el mismo nodo como inicio y objetivo

    path = bfs_search(start_node, goal_node)
    if path is not None:
        # Actualizar matriz de recompensas
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            reward_matrix[current_node][next_node] += 1

# Entrenamiento del agente utilizando aprendizaje por refuerzo pasivo
def passive_reinforcement_learning(num_iterations):
    for _ in range(num_iterations):
        start_node = random.choice(list(graph.keys()))
        explore_and_update(start_node)

# Ejecutar el aprendizaje por refuerzo pasivo
passive_reinforcement_learning(num_iterations=1000)

# Imprimir la matriz de recompensas actualizada
print("Matriz de Recompensas:")
for node in graph:
    print(f"Desde nodo {node}: {reward_matrix[node]}")
