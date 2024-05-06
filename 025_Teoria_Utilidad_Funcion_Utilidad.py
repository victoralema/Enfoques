#Victor Eduardo Aleman Padilla 21310193


# Definición de la clase para los nodos del grafo
class Node:
    def __init__(self, state, utility=0):
        self.state = state  # Estado del nodo
        self.utility = utility  # Utilidad del nodo

# Función para expandir un nodo y generar sus sucesores
def expand(node):
    # En este ejemplo, los sucesores se generan de forma arbitraria
    # Aquí se debería implementar la lógica para generar los sucesores
    successors = [
        Node(state="Estado_1", utility=4),
        Node(state="Estado_2", utility=6),
        Node(state="Estado_3", utility=2)
    ]
    return successors

# Algoritmo de búsqueda en grafos con teoría de la utilidad
def graph_search_with_utility(start_node):
    frontier = [start_node]  # Lista de nodos frontera
    explored = set()  # Conjunto de nodos explorados

    while frontier:
        node = frontier.pop(0)  # Seleccionar el nodo de la frontera
        if node.state not in explored:
            explored.add(node.state)  # Marcar el nodo como explorado
            if goal_test(node):  # Verificar si el nodo es objetivo
                return node
            # Expandir el nodo y agregar sus sucesores a la frontera
            successors = expand(node)
            for successor in successors:
                frontier.append(successor)

# Función para verificar si un nodo es objetivo (depende del problema)
def goal_test(node):
    # En este ejemplo, el objetivo es encontrar un estado con utilidad mayor a 5
    return node.utility > 5

# Ejemplo de uso
if __name__ == "__main__":
    # Crear los nodos del grafo
    start_node = Node(state="Inicio", utility=3)
    # Ejecutar la búsqueda en el grafo con teoría de la utilidad
    result_node = graph_search_with_utility(start_node)
    print("Estado objetivo encontrado:", result_node.state)
    print("Utilidad del estado objetivo:", result_node.utility)
