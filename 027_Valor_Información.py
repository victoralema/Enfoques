#Victor Eduardo Aleman Padilla 21310193

# Definición de la clase para los nodos del grafo
class Node:
    def __init__(self, state, value=0):
        self.state = state  # Estado del nodo
        self.value = value  # Valor del nodo
        self.children = {}  # Diccionario de nodos hijos y sus probabilidades de transición

# Función para expandir un nodo y generar sus sucesores
def expand(node):
    # En este ejemplo, los sucesores se generan de forma arbitraria
    # Aquí se debería implementar la lógica para generar los sucesores
    successors = {
        "A": Node(state="A", value=5),
        "B": Node(state="B", value=3),
        "C": Node(state="C", value=7)
    }
    # Asignar probabilidades de transición a los sucesores
    for successor in successors.values():
        successor.probability = 1 / len(successors)
    return successors

# Algoritmo de búsqueda en grafos con Valor de la Información
def graph_search_with_value_of_information(start_node):
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
            for successor in successors.values():
                frontier.append(successor)

# Función para verificar si un nodo es objetivo (depende del problema)
def goal_test(node):
    # En este ejemplo, el objetivo es encontrar un estado con valor mayor a 6
    return node.value > 6

# Ejemplo de uso
if __name__ == "__main__":
    # Crear el nodo inicial del grafo
    start_node = Node(state="Inicio", value=0)
    # Ejecutar la búsqueda en el grafo con Valor de la Información
    result_node = graph_search_with_value_of_information(start_node)
    print("Estado objetivo encontrado:", result_node.state)
    print("Valor del estado objetivo:", result_node.value)
