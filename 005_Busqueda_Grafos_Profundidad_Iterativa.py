def depth_limited_search(graph, start, goal, depth_limit):# Función para realizar la Búsqueda en Profundidad Limitada (DLS)
    visited = set()  # Conjunto para mantener los nodos visitados
    return dls_recursive(graph, start, goal, visited, depth_limit, 0)

def dls_recursive(graph, current_node, goal, visited, depth_limit, current_depth):# Función auxiliar para realizar la Búsqueda en Profundidad Limitada (DLS) de manera recursiva
    if current_node == goal:
        return True  # Se encontró el objetivo, retornar True
    
    if current_depth >= depth_limit:
        return False  # Límite de profundidad alcanzado sin encontrar el objetivo, retornar False
    
    visited.add(current_node)  # Marcar el nodo actual como visitado
    
    for neighbor in graph[current_node]:
        if neighbor not in visited:
            # Llamada recursiva para explorar el nodo vecino
            if dls_recursive(graph, neighbor, goal, visited, depth_limit, current_depth + 1):
                return True  # Si se encontró el objetivo en la llamada recursiva, retornar True
    
    return False  # No se encontró el objetivo dentro del límite de profundidad

def iterative_deepening_search(graph, start, goal):# Función para realizar la Búsqueda en Profundidad Iterativa (IDS)
    depth_limit = 0 # Inicializa el límite de profundidad (depth_limit) en cero antes de comenzar la iteración.
    
    while True:# Aplicar Búsqueda en Profundidad Limitada con incremento en el límite de profundidad
        if depth_limited_search(graph, start, goal, depth_limit): # Llama a la función depth_limited_search para realizar la búsqueda en profundidad limitada con el depth_limit actual.
            return depth_limit  # Se encontró el objetivo dentro del límite de profundidad
        
        depth_limit += 1  # Incrementar el límite de profundidad e intentar nuevamente

# Ejemplo de uso
if __name__ == "__main__": # Se define un grafo graph como un diccionario de listas de adyacencia que representa las conexiones entre nodos.
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
    goal_node = 'I'  # se especifican el goal_node


    result = iterative_deepening_search(graph, start_node, goal_node) # Llamar a la función de Búsqueda en Profundidad Iterativa (IDS)

    if result is not None:
        print("Se encontró un camino desde {start_node} hasta {goal_node} con una profundidad máxima de {result}.")  # Imprime el resultado de la búsqueda, mostrando si se encontró un camino y la profundidad máxima alcanzada si se encontró un camino válido.
    else:
        print("No se encontró un camino desde {start_node} hasta {goal_node}.")
