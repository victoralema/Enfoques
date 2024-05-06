#Victor Eduardo Aleman Padilla 21310193

import heapq #Importa el módulo heapq, que proporciona 

def heuristic(node, goal): #Define una función heuristic que calcula una heurística entre dos nodos. En este caso, se utiliza la distancia Euclidiana como heurística para estimar la distancia entre node y goal.
    # Ejemplo de heurística: distancia Euclidiana en un grafo no ponderado
    return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

def greedy_best_first_search(graph, start, goal):  #Define la función principal greedy_best_first_search que implementa el algoritmo de Búsqueda Voraz. 
    # Inicializar la cola de prioridad (heap) con el nodo de inicio y su valor heurístico
    priority_queue = [(heuristic(start, goal), start)]  #Inicializa la cola de prioridad (priority_queue) con el nodo de inicio y su valor heurístico calculado utilizando la función heuristic.
    heapq.heapify(priority_queue) #Convierte la lista priority_queue en una cola de prioridad (heap) utilizando la función heapify del módulo heapq.
    
    # Conjunto para mantener un registro de nodos visitados
    visited = set()

    # Mientras haya nodos en la cola de prioridad
    while priority_queue:
        # Obtener el nodo con el menor valor heurístico (más prometedor)
        _, current_node = heapq.heappop(priority_queue)
        
        # Si llegamos al nodo objetivo, retornar True
        if current_node == goal:
            return True
        
        # Marcar el nodo como visitado
        visited.add(current_node)
        
        # Explorar los nodos vecinos del nodo actual
        for neighbor in graph.get(current_node, []):
            if neighbor not in visited: #Verifica si el vecino no ha sido visitado previamente.
                # Calcular el valor heurístico del vecino y agregarlo a la cola de prioridad
                priority = heuristic(neighbor, goal)
                heapq.heappush(priority_queue, (priority, neighbor)) #Agrega el vecino a la cola de prioridad con su valor heurístico como clave para mantener el orden.
    
    # Si se agota la cola de prioridad y no se encontró el nodo objetivo, retornar False
    return False

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

    # Llamar a la función de Búsqueda Voraz para encontrar si hay camino hacia el nodo objetivo
    if greedy_best_first_search(graph, start_node, goal_node):
        print("Se encontró un camino desde {start_node} hasta {goal_node}.")
    else:
        print("No se encontró un camino desde {start_node} hasta {goal_node}.")
