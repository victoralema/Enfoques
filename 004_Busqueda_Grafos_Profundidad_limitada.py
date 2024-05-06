def depth_limited_search(graph, start, goal, depth_limit):  # Esta línea define una función llamada depth_limited_search que toma cuatro parámetros
    visited = set()  # Conjunto para mantener los nodos visitados
    return dls_recursive(graph, start, goal, visited, depth_limit, 0) # Llama a la función dls_recursive con los parámetros iniciales para iniciar la búsqueda en profundidad limitada desde el nodo de inicio start con una profundidad actual de 0.

def dls_recursive(graph, current_node, goal, visited, depth_limit, current_depth): # Define la función auxiliar dls_recursive que realiza la Búsqueda en Profundidad Limitada (DLS) de manera recursiva.
    # Verificar si el nodo actual es el objetivo
    if current_node == goal: # Verifica si el current_node es igual al goal. Si es así, se ha encontrado el objetivo y se devuelve True.
        return True  # Se encontró el objetivo
    
    # Verificar si se ha alcanzado el límite de profundidad
    if current_depth >= depth_limit: # erifica si la current_depth ha alcanzado el depth_limit. Si es así, se detiene la búsqueda en esta rama y se devuelve False.
        return False  # Límite de profundidad alcanzado sin encontrar el objetivo
    
    visited.add(current_node)  # Marcar el nodo como visitado
    
    # Explorar los nodos vecinos recursivamente
    for neighbor in graph[current_node]: # Itera sobre cada vecino (neighbor) del current_node en el grafo.
        if neighbor not in visited: # Verifica si el vecino no ha sido visitado.
            # Llamada recursiva con incremento en la profundidad
            if dls_recursive(graph, neighbor, goal, visited, depth_limit, current_depth + 1):  # Llama recursivamente a dls_recursive para explorar el vecino (neighbor) con una profundidad incrementada en 1.
                return True  # Si se encontró el objetivo en la llamada recursiva, retornar True
    
    return False  # No se encontró el objetivo dentro del límite de profundidad

# Ejemplo de uso
if __name__ == "__main__":  # Verifica si el script está siendo ejecutado como programa principal.
    # Definir un grafo como un diccionario de listas de adyacencia
    graph = {
        'A': ['B', 'C'],
        'B': ['D', 'E'],
        'C': ['F'],
        'D': ['G'],
        'E': [],
        'F': ['H', 'I'],
        'G': [],
        'H': [],
        'I': []
    }

    start_node = 'A' #se especifican el start_node
    goal_node = 'I'  #se especifican el goal_node
    depth_limit = 3  # Límite de profundidad

    # Llamar a la función de búsqueda en profundidad limitada
    if depth_limited_search(graph, start_node, goal_node, depth_limit): # Finalmente, se llama a depth_limited_search con los parámetros definidos y se imprime si se encontró un camino dentro del límite de profundidad o no.
        print(f"Se encontró un camino desde {start_node} hasta {goal_node} dentro del límite de profundidad {depth_limit}.")
    else:
        print(f"No se encontró un camino desde {start_node} hasta {goal_node} dentro del límite de profundidad {depth_limit}.")
