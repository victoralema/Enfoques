#Victor Eduardo Aleman Padilla 21310193

import heapq # Importa el módulo heapq, que proporciona funciones para implementar colas de prioridad (heaps) en Python.

def heuristic(node, goal):  #Define una función heuristic que calcula una heurística entre dos nodos. En este caso, utiliza la distancia Manhattan para estimar el costo desde node hasta goal.
    # Ejemplo de heurística: distancia Manhattan en un grafo no ponderado
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def a_star(graph, start, goal):  #Define la función principal a_star que implementa el algoritmo A*. Toma como entrada el grafo (graph), el nodo de inicio (start) y el nodo objetivo (goal).
    # Inicializar la cola de prioridad (heap) con el nodo de inicio y su costo estimado (f = g + h)
    priority_queue = [(heuristic(start, goal), start)]  #Inicializa la cola de prioridad (priority_queue) con el nodo de inicio y su costo estimado (f = g + h) utilizando la función heurística.
    heapq.heapify(priority_queue)  #Convierte la lista priority_queue en una cola de prioridad (heap) utilizando la función heapify del módulo heapq.
    
    # Diccionarios para registrar el costo real (g) y el nodo padre
    g = {start: 0} #Inicializa los diccionarios
    parent = {start: None} #mantener el nodo padre de cada nodo visitado.

    # Mientras haya nodos en la cola de prioridad
    while priority_queue:
        # Obtener el nodo con el menor costo estimado (f = g + h)
        current_cost, current_node = heapq.heappop(priority_queue)  #Extrae el nodo con el menor costo estimado (current_cost) de la cola de prioridad.
        
        # Si llegamos al nodo objetivo, reconstruir el camino y retornarlo
        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parent[current_node]
            path.reverse()
            return path #Retorna el camino reconstruido desde el nodo objetivo hasta el nodo inicial.
        
        # Explorar los nodos vecinos del nodo actual
        for neighbor in graph.get(current_node, []):
            # Calcular el costo real acumulado desde el nodo inicial hasta el vecino
            new_cost = g[current_node] + 1  # En un grafo no ponderado, el costo de cada paso es 1
            
            # Si es la primera vez que visitamos este vecino o encontramos un camino más corto
            if neighbor not in g or new_cost < g[neighbor]:
                # Actualizar el costo real y el costo estimado (f = g + h) del vecino
                g[neighbor] = new_cost  #Actualiza el costo real acumulado para el vecino.
                priority = new_cost + heuristic(neighbor, goal) #Calcula el nuevo costo estimado (f = g + h) para el vecino utilizando la función heurística.
                heapq.heappush(priority_queue, (priority, neighbor))  #Agrega el vecino a la cola de prioridad con su nuevo costo estimado.
                parent[neighbor] = current_node #Registra el nodo actual como el padre del vecino.
    
    # Si no se encontró un camino hacia el nodo objetivo
    return None

# Ejemplo de uso
if __name__ == "__main__":
    # Definir el grafo como un diccionario de listas de adyacencia (grafo no ponderado)
    graph = {
        (0, 0): [(0, 1), (1, 0)],
        (0, 1): [(0, 0), (1, 1)],
        (1, 0): [(0, 0), (1, 1)],
        (1, 1): [(0, 1), (1, 0)]
    }

    start_node = (0, 0)  # Nodo de inicio
    goal_node = (1, 1)    # Nodo objetivo

    # Llamar a la función A* para encontrar el camino desde el nodo inicial hasta el nodo objetivo
    path = a_star(graph, start_node, goal_node)

    if path:
        print(f"Se encontró un camino desde {start_node} hasta {goal_node}: {path}")
    else:
        print(f"No se encontró un camino desde {start_node} hasta {goal_node}.")
