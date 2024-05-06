#Victor Eduardo Aleman Padilla 21310193
# Importar la clase deque desde el módulo collections
from collections import deque

# Definir la función bfs que toma como argumentos el grafo (representado como un diccionario de listas de adyacencia), el nodo de inicio y el nodo objetivo
def bfs(graph, start, goal):
    # Cola para el BFS
    queue = deque([start])
    # Conjunto para mantener un registro de nodos visitados
    visited = set([start])

    # Mientras haya nodos por visitar en la cola
    while queue:
        # Sacar el nodo de la cola
        current_node = queue.popleft()
        
        # Si el nodo actual es el nodo objetivo, retornar True
        if current_node == goal:
            return True
        
        # Iterar sobre los nodos vecinos del nodo actual
        for neighbor in graph.get(current_node, []):
            # Si el vecino no ha sido visitado
            if neighbor not in visited:
                # Marcar el nodo vecino como visitado y agregarlo a la cola
                visited.add(neighbor)
                queue.append(neighbor)
    
    # Si llegamos aquí, el nodo objetivo no fue encontrado
    return False

# Ejemplo de uso
if __name__ == "__main__":
    # Definir el grafo como un diccionario de listas de adyacencia
    graph = {
        'A': ['B', 'C'],
        'B': ['A', 'D', 'E'],
        'C': ['A', 'F'],
        'D': ['B'],
        'E': ['B', 'F'],
        'F': ['C', 'E']
    }

    start_node = 'A'  # Nodo de inicio
    goal_node = 'F'   # Nodo objetivo

    # Llamar a la función BFS para buscar el nodo objetivo desde el nodo inicial
    if bfs(graph, start_node, goal_node):
        print(f"Se encontró un camino desde {start_node} hacia {goal_node}.")
    else:
        print(f"No se encontró un camino desde {start_node} hacia {goal_node}.")
