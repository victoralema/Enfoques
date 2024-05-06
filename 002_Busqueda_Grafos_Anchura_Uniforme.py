import heapq   # Importa el módulo heapq, que proporciona funciones para implementar colas de prioridad (heaps) en Python.

def uniform_cost_search(graph, start, goal): # Define una función llamada uniform_cost_search que toma tres argumentos
    priority_queue = [(0, start)]  # Inicializa una cola de prioridad (priority_queue) con una tupla que contiene el costo acumulado inicial (0) y el nodo de inicio (start).

    costs = {start: 0}   # Diccionario para mantener el costo mínimo conocido para cada nodo

    while priority_queue:  # Inicia un bucle while que continúa mientras la cola de prioridad no esté vacía.
        current_cost, current_node = heapq.heappop(priority_queue)  # Sacar el nodo con el menor costo acumulado

        if current_node == goal: # Verifica si el current_node es igual al nodo objetivo (goal).
            print("Ruta encontrada: {reconstruct_path(start, goal, parent_map)}") # imprime la ruta encontrada.
            return current_cost  # Devolver el costo acumulado de la ruta encontrada

        for neighbor, weight in graph[current_node].items():    # Itera sobre los vecinos (neighbor) y los pesos (weight) del nodo actual (current_node) en el grafo.
            new_cost = current_cost + weight   #Calcula el nuevo costo acumulado para llegar al vecino desde el nodo actual.

            if neighbor not in costs or new_cost < costs[neighbor]:   #Verifica si el vecino aún no tiene un costo asignado o si se encontró un camino más corto hacia el vecino.
                costs[neighbor] = new_cost  # Actualiza el costo mínimo conocido para el vecino.
                heapq.heappush(priority_queue, (new_cost, neighbor))    # Agrega el vecino y su nuevo costo acumulado a la cola de prioridad.

    return float('inf')  # Si no se encuentra ninguna ruta al nodo objetivo, devolver infinito

def reconstruct_path(start, goal, parent_map): # Define una función llamada reconstruct_path que reconstruye la ruta desde el nodo objetivo (goal) hasta el nodo inicial (start) utilizando un diccionario parent_map que almacena los padres de cada nodo en el camino más corto.
    # Reconstruir la ruta desde el nodo objetivo hasta el nodo inicial
    path = []
    current = goal  #nicializa una lista path para almacenar la ruta y comienza desde el nodo objetivo (goal).

    while current != start:   #Itera mientras el nodo actual no sea igual al nodo inicial.
        path.append(current)   # Agrega el nodo actual a la ruta y actualiza el nodo actual al padre del nodo actual.
        current = parent_map[current]

    path.append(start)   #Agrega el nodo inicial a la ruta y revierte la lista para obtener la ruta desde el nodo inicial hasta el nodo objetivo.
    path.reverse()
    return path

# Ejemplo de uso
if __name__ == "__main__":  #Comprueba si el script se está ejecutando como un programa principal.
    # Definir un grafo ponderado como un diccionario de diccionarios
    # Cada nodo tiene un diccionario de nodos vecinos y sus respectivos pesos
    graph = {
        'A': {'B': 4, 'C': 2},
        'B': {'A': 4, 'D': 5},
        'C': {'A': 2, 'D': 8},
        'D': {'B': 5, 'C': 8}
    }

    start_node = 'A'
    goal_node = 'D'

    # Llamar a la función uniform_cost_search para encontrar la ruta más corta
    min_cost = uniform_cost_search(graph, start_node, goal_node) #Llama a la función uniform_cost_search para encontrar la ruta más corta desde start_node hasta goal_node en el grafo graph.

    if min_cost != float('inf'):  #Comprueba si se encontró una ruta válida (costo mínimo diferente de infinito).
        print("Costo mínimo desde {start_node} hasta {goal_node}: {min_cost}")  # Imprime el costo mínimo de la ruta encontrada.
    else:   #Si no se encontró una ruta válida.
        print("No se encontró ninguna ruta desde {start_node} hasta {goal_node}") # Imprime un mensaje indicando que no se encontró ninguna ruta desde start_node hasta goal_node.
