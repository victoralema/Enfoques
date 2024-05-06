#Victor Eduardo Aleman Padilla 21310193

class Graph:
    def __init__(self):
        self.graph = {}  # Inicializa un diccionario vacío para representar el grafo

    def add_edge(self, node1, node2):
        if node1 not in self.graph:
            self.graph[node1] = []  # Crea una lista vacía para los vecinos de node1 si node1 no está en el grafo
        if node2 not in self.graph:
            self.graph[node2] = []  # Crea una lista vacía para los vecinos de node2 si node2 no está en el grafo
        self.graph[node1].append(node2)  # Agrega node2 a la lista de vecinos de node1
        self.graph[node2].append(node1)  # Agrega node1 a la lista de vecinos de node2 (grafo no dirigido)

    def forward_checking_search(self, current_node, target_node, path=[]):
        path = path + [current_node]  # Agrega el nodo actual al camino

        if current_node == target_node:
            return path  # Si el nodo actual es el nodo objetivo, devuelve el camino encontrado

        for neighbor in self.graph[current_node]:  # Itera sobre los vecinos del nodo actual
            if neighbor not in path:  # Verifica si el vecino no está en el camino actual
                # Realiza una búsqueda recursiva desde el vecino
                result = self.forward_checking_search(neighbor, target_node, path)
                if result is not None:  # Si se encontró un camino hacia el nodo objetivo
                    return result  # Devuelve el camino encontrado

        return None  # Indica que no se encontró un camino válido

# Ejemplo de uso
if __name__ == "__main__":
    graph = Graph()  # Crea una instancia de la clase Graph para representar el grafo

    # Definir conexiones (aristas) entre nodos
    graph.add_edge('A', 'B')
    graph.add_edge('A', 'C')
    graph.add_edge('B', 'D')
    graph.add_edge('C', 'E')
    graph.add_edge('D', 'E')
    graph.add_edge('D', 'F')
    graph.add_edge('E', 'F')

    # Nodo inicial y nodo objetivo
    start_node = 'A'
    goal_node = 'F'

    # Resolver el problema utilizando búsqueda con comprobación hacia delante
    path = graph.forward_checking_search(start_node, goal_node)

    if path is not None:
        print("Camino encontrado:")
        print(" -> ".join(path))  # Imprime el camino encontrado como una cadena
    else:
        print("No se encontró un camino desde {} hasta {}.".format(start_node, goal_node))
