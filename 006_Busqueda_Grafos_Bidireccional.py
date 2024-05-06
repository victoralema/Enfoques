from collections import deque # Importa la clase deque de la biblioteca collections, que se utiliza para implementar colas eficientes.

def bidirectional_search(graph, start, goal): # Define la función bidirectional_search que realiza la Búsqueda Bidireccional en un grafo dado, con los parámetros graph (grafo representado como un diccionario de listas de adyacencia), start (nodo de inicio) y goal (nodo objetivo).
    if start == goal: # Verifica si el nodo de inicio es igual al nodo objetivo. Si son iguales, devuelve una lista que contiene solo el nodo de inicio como el camino trivial.
        return [start]  # El nodo de inicio es igual al nodo objetivo
    
    # Cola de nodos para la búsqueda desde el nodo inicial
    queue_start = deque([start])
    visited_start = set([start])
    
    # Cola de nodos para la búsqueda desde el nodo objetivo
    queue_goal = deque([goal]) 
    visited_goal = set([goal])
    
    # Diccionario para registrar el camino desde cada nodo
    predecessor_start = {start: None}
    predecessor_goal = {goal: None}
    
    while queue_start and queue_goal: # Inicia un bucle mientras ambas colas queue_start y queue_goal no estén vacías, lo que indica que aún hay nodos por explorar desde ambos extremos.
        # Búsqueda desde el nodo inicial
        current_start = queue_start.popleft() # Se extrae el primer nodo de la cola queue_start y se exploran sus vecinos en el grafo.
        for neighbor in graph[current_start]:
            if neighbor not in visited_start:
                visited_start.add(neighbor) # mantener registro de los nodos visitados desde el nodo inicial
                queue_start.append(neighbor) # Inicializa conjuntos visited_start para mantener registro de los nodos visitados desde el nodo inicial
                predecessor_start[neighbor] = current_start # Inicializa diccionarios predecessor_start para registrar el predecesor de cada nodo en el camino desde el nodo inicial 
                # Verificar si se encuentra en la otra búsqueda
                if neighbor in visited_goal:
                    return reconstruct_path(predecessor_start, predecessor_goal, neighbor) # para reconstruir y devolver el camino completo desde el nodo inicial hasta el nodo objetivo
        
        # Búsqueda desde el nodo objetivo
        current_goal = queue_goal.popleft() # Similar a la búsqueda desde el nodo inicial, pero explorando los vecinos desde el nodo objetivo.
        for neighbor in graph[current_goal]:
            if neighbor not in visited_goal:
                visited_goal.add(neighbor)  # mantener registro de los nodos visitados desde el nodo nodo objetivo respectivamente.
                queue_goal.append(neighbor) # Inicializa conjuntos visited_goal para mantener registro de los nodos visitados desde el nodo objetivo respectivamente.
                predecessor_goal[neighbor] = current_goal # Inicializa diccionarios predecessor_goal para registrar el predecesor de cada nodo en el camino desde el nodo objetivo respectivamente.
                # Verificar si se encuentra en la otra búsqueda
                if neighbor in visited_start:
                    return reconstruct_path(predecessor_start, predecessor_goal, neighbor) # permite reconstruir completamente el camino más corto desde el nodo inicial hasta el nodo objetivo,
    
    return None  # No se encontró un camino

def reconstruct_path(predecessor_start, predecessor_goal, intersection):# Reconstruir el camino combinando los predecesores desde ambos extremos
    path = []
    current = intersection
    
    # Construir camino desde el nodo inicial al nodo intersección
    while current is not None:
        path.append(current) # Agrega el nodo current al camino path. En este contexto, current representa el nodo actual en el proceso de reconstrucción del camino desde el nodo de intersección hacia el nodo objetivo.
        current = predecessor_start[current] # Actualiza current asignándole el predecesor del nodo current en la búsqueda desde el nodo objetivo. 
    
    path.reverse()  # Invertir el camino para que comience desde el inicio
    
    current = predecessor_goal[intersection] # Asigna este nodo predecesor a la variable current. 
    
    # Agregar el camino desde el nodo objetivo al nodo intersección
    while current is not None:
        path.append(current) # Agrega el nodo current al camino path. En este contexto, current representa el nodo actual en el proceso de reconstrucción del camino desde el nodo de intersección hacia el nodo objetivo.
        current = predecessor_goal[current] # Actualiza current asignándole el predecesor del nodo current en la búsqueda desde el nodo objetivo. 
    
    return path

# Ejemplo de uso
if __name__ == "__main__": # Define un grafo graph como un diccionario de listas de adyacencia que representa las conexiones entre nodos.
    # Definir un grafo como un diccionario de listas de adyacencia
    graph = {
        'A': ['B', 'C'],
        'B': ['A', 'D', 'E'],
        'C': ['A', 'F'],
        'D': ['B'],
        'E': ['B', 'F'],
        'F': ['C', 'E']
    }

    start_node = 'A'
    goal_node = 'F'

    # Llamar a la función de búsqueda bidireccional
    path = bidirectional_search(graph, start_node, goal_node) # Llama a la función bidirectional_search con el grafo graph, el nodo de inicio start_node y el nodo objetivo goal_node para buscar el camino más corto entre ambos nodos.

    if path: # Verifica si se encontró un camino (path no es None).
        print("Se encontró un camino entre {start_node} y {goal_node}: {path}") # Si se encontró un camino, imprime el camino encontrado entre el nodo inicial y el nodo objetivo.
    else:
        print("No se encontró un camino entre {start_node} y {goal_node}.") # Si no se encontró un camino, imprime un mensaje indicando que no se encontró un camino válido entre los nodos.
