#Victor Eduardo Aleman Padilla 21310193
import heapq  # Importa el módulo heapq para trabajar con colas de prioridad (frontera de búsqueda)

# Definición del grafo del mundo con nodos (ubicaciones) y aristas (acciones entre ubicaciones)
graph = {
    'A': {'B': 1, 'C': 3},  # Desde el nodo A, hay aristas a B con costo 1 y a C con costo 3
    'B': {'D': 2},           # Desde el nodo B, hay una arista a D con costo 2
    'C': {'D': 1},           # Desde el nodo C, hay una arista a D con costo 1
    'D': {'G': 2},           # Desde el nodo D, hay una arista a G con costo 2
    'E': {'F': 2},           # Desde el nodo E, hay una arista a F con costo 2
    'F': {'G': 1}            # Desde el nodo F, hay una arista a G con costo 1
}

# Función de búsqueda A* para encontrar el camino más corto desde el inicio hasta el objetivo
def astar(start, goal):
    # Frontera de búsqueda prioritaria (cola de prioridad) inicializada con el nodo de inicio y su costo 0
    frontier = [(0, start)]
    # Diccionario para almacenar los costos reales desde el inicio hasta cada nodo (inicialmente solo el nodo de inicio con costo 0)
    real_costs = {start: 0}
    
    # Mientras haya nodos en la frontera
    while frontier:
        # Extraer el nodo con el menor costo estimado de la frontera
        current_cost, current_node = heapq.heappop(frontier)
        
        # Si llegamos al objetivo, terminamos la búsqueda
        if current_node == goal:
            return current_cost
        
        # Iterar sobre los vecinos del nodo actual
        for neighbor, cost in graph[current_node].items():
            # Calcular el costo estimado desde el inicio hasta el objetivo a través de este vecino
            estimated_cost = current_cost + cost + heuristic(neighbor, goal)
            
            # Si el vecino no está en el diccionario de costos reales o si el costo estimado es menor
            if neighbor not in real_costs or estimated_cost < real_costs[neighbor]:
                # Actualizar el costo real del vecino
                real_costs[neighbor] = estimated_cost
                # Agregar el vecino a la frontera con su costo estimado como prioridad
                heapq.heappush(frontier, (estimated_cost, neighbor))
    
    # Si no se encontró un camino al objetivo
    return None

# Función heurística (en este caso, la distancia euclidiana entre dos ubicaciones)
# Función heurística (en este caso, una función simple que devuelve 0)
def heuristic(node, goal):
    # Como estamos usando nombres de ubicaciones como nodos en lugar de coordenadas, simplemente devolvemos 0
    return 0

# Ejemplo de uso
start = 'A'  # Nodo de inicio
goal = 'G'   # Nodo objetivo
cost = astar(start, goal)  # Realizar la búsqueda A* para encontrar el camino más corto
if cost is not None:
    print(f"El costo del camino más corto desde {start} hasta {goal} es: {cost}")  # Imprimir el costo del camino más corto
else:
    print(f"No se encontró un camino desde {start} hasta {goal}")  # Imprimir un mensaje si no se encontró un camino
