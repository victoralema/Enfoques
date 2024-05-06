#Victor Eduardo Aleman Padilla 21310193

import random #Importa el módulo random de Python, que se utiliza para realizar selecciones aleatorias.

class GraphColoring: #Define una clase llamada GraphColoring para resolver el problema de coloración de grafos.
    def __init__(self, graph, max_steps=1000):
        self.graph = graph  # Grafo representado como un diccionario de nodos y sus vecinos
        self.colors = {}  # Diccionario para almacenar los colores asignados a cada nodo
        self.max_steps = max_steps  # Número máximo de iteraciones permitidas

    def assign_initial_colors(self): #Define un método para asignar colores iniciales aleatorios a cada nodo del grafo.
        """ Asigna un color aleatorio a cada nodo del grafo """
        for node in self.graph: #Itera sobre cada nodo en el grafo.
            self.colors[node] = random.choice(['Red', 'Green', 'Blue'])  # Asigna un color aleatorio

    def count_conflicts(self, node, color): #Define un método para contar el número de conflictos si se asigna cierto color a un nodo dado.
        """ Calcula el número de conflictos para un nodo dado si se le asigna cierto color """
        conflicts = 0
        for neighbor in self.graph[node]: #Itera sobre los vecinos del nodo.
            if self.colors.get(neighbor) == color: #Comprueba si el vecino tiene el mismo color que el color especificado.
                conflicts += 1
        return conflicts

    def minimize_conflicts(self, node): #Define un método para encontrar el color que minimiza el número de conflictos para un nodo dado.
        """ Encuentra el color que minimiza el número de conflictos para un nodo """
        min_conflicts = float('inf')
        min_color = None

        current_color = self.colors[node]

        # Evalúa cada color diferente para el nodo actual y elige el que minimice los conflictos
        for color in ['Red', 'Green', 'Blue']: #Itera sobre cada color posible ('Rojo', 'Verde', 'Azul').
            if color != current_color: #Evita seleccionar el color actual del nodo.
                conflicts = self.count_conflicts(node, color)
                if conflicts < min_conflicts:
                    min_conflicts = conflicts
                    min_color = color

        return min_color

    def local_search(self): #Define un método para ejecutar la búsqueda local con mínimos conflictos.
        """ Ejecuta la búsqueda local con mínimos conflictos """
        self.assign_initial_colors()  # Asigna colores iniciales aleatorios

        for _ in range(self.max_steps): # Itera hasta alcanzar el número máximo de pasos.
            conflict_nodes = [] #Inicializa una lista vacía para almacenar los nodos con conflictos.

            # Encuentra todos los nodos con conflictos
            for node in self.graph: #Itera sobre cada nodo en el grafo.
                if self.has_conflict(node): #Verifica si el nodo tiene conflictos utilizando el método has_conflict.
                    conflict_nodes.append(node)

            if not conflict_nodes: #Comprueba si no hay nodos con conflictos.
                return True  # No hay conflictos, se encontró una solución

            # Elige un nodo con conflictos al azar
            node_to_fix = random.choice(conflict_nodes)

            # Minimiza los conflictos para el nodo seleccionado
            new_color = self.minimize_conflicts(node_to_fix)
            self.colors[node_to_fix] = new_color

        return False  # No se encontró una solución válida dentro del número máximo de iteraciones

    def has_conflict(self, node):
        """ Verifica si un nodo tiene conflictos con algún vecino """
        current_color = self.colors[node]
        for neighbor in self.graph[node]:
            if self.colors.get(neighbor) == current_color:
                return True
        return False

# Ejemplo de uso
if __name__ == "__main__":
    # Definir el grafo como un diccionario de nodos y sus vecinos
    graph = {
        'A': ['B', 'C'],
        'B': ['A', 'C', 'D'],
        'C': ['A', 'B', 'D'],
        'D': ['B', 'C']
    }

    # Crear una instancia del problema de coloración de grafos
    problem = GraphColoring(graph)

    # Resolver el problema utilizando búsqueda local con mínimos conflictos
    solution_found = problem.local_search()

    if solution_found: #Comprueba si se encontró una solución válida.
        print("Asignación de colores encontrada:")
        for node, color in problem.colors.items(): #Itera sobre cada nodo y su color asignado.
            print(f"Nodo {node}: Color {color}")
    else:
        print("No se encontró una asignación válida de colores dentro del límite de iteraciones.")
