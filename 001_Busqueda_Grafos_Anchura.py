from collections import deque

def bfs(graph, start):              # La función bfs toma dos argumentos
    visited = set()                 # Conjunto para almacenar nodos visitados
    queue = deque([start])          # Cola para el BFS, comienza con el nodo inicial

    while queue:                    # El bucle while continúa mientras haya nodos en la cola.
        node = queue.popleft()      # Sacar el nodo de la cola
        if node not in visited:
            print(node)             # Procesar el nodo (aquí solo lo imprimimos)
            visited.add(node)       # Marcar el nodo como visitado

            for neighbor in graph[node]:  # Agregar vecinos no visitados a la cola
                if neighbor not in visited:
                    queue.append(neighbor)  # Inicia una cola queue 

# Ejemplo de uso
if __name__ == "__main__":
    # Definir un grafo como un diccionario de listas de adyacencia
    graph = {
        'A': ['B', 'C'],
        'B': ['A', 'D', 'E'],
        'C': ['A', 'F'],
        'D': ['B'],
        'E': ['B', 'F'],
        'F': ['C', 'E']
    }

    # Llamar a la función bfs con el nodo inicial 'A'
    print("Recorrido BFS comenzando desde el nodo 'A':") # Imprimimos texto de informacion
    bfs(graph, 'A')          # Esto imprimirá el recorrido BFS empezando desde el nodo 'A'.
