#Victor Eduardo Aleman Padilla 21310193

import heapq

class Graph:
    def __init__(self):
        self.adjacency_list = {}

    def add_edge(self, u, v, weight):
        if u not in self.adjacency_list:
            self.adjacency_list[u] = []
        self.adjacency_list[u].append((v, weight))

    def a_star(self, start, goal):
        open_set = []  # Min-heap para mantener los nodos por explorar
        heapq.heappush(open_set, (0, start))  # Tupla (f, nodo): f = g + h, inicialmente g = 0

        came_from = {}  # Diccionario para reconstruir el camino

        # Costos reales acumulados desde el inicio hasta cada nodo
        g_scores = {node: float('inf') for node in self.adjacency_list}
        g_scores[start] = 0

        # Función heurística: distancia Euclidiana entre dos nodos en un espacio 2D
        def heuristic(node, goal):
            return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruir el camino desde el objetivo hasta el inicio
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor, weight in self.adjacency_list.get(current, []):
                tentative_g_score = g_scores[current] + weight
                if tentative_g_score < g_scores[neighbor]:
                    came_from[neighbor] = current
                    g_scores[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None  # No se encontró un camino

# Ejemplo de uso
if __name__ == "__main__":
    graph = Graph()

    # Agregar aristas al grafo (nodos y pesos)
    graph.add_edge('A', 'B', 3)
    graph.add_edge('A', 'C', 5)
    graph.add_edge('B', 'C', 2)
    graph.add_edge('B', 'D', 6)
    graph.add_edge('C', 'D', 1)
    graph.add_edge('C', 'E', 8)
    graph.add_edge('D', 'E', 4)

    # Definir nodo de inicio y nodo objetivo
    start_node = 'A'
    goal_node = 'E'

    # Aplicar el algoritmo A* para encontrar el camino más corto
    path = graph.a_star(start_node, goal_node)

    if path:
        print(f"Camino encontrado de {start_node} a {goal_node}: {path}")
        total_cost = sum(graph.adjacency_list[node][1] for node in path[:-1])
        print(f"Costo total del camino: {total_cost}")
    else:
        print(f"No se encontró un camino de {start_node} a {goal_node}.")
