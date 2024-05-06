#Victor Eduardo Aleman Padilla 21310193

# Definición de la clase para los nodos del grafo
class Node:
    def __init__(self, state, value=0):
        self.state = state  # Estado del nodo
        self.value = value  # Valor del nodo
        self.successors = []  # Lista de nodos sucesores

# Algoritmo de Iteración de Valores
def value_iteration(graph, threshold=0.01, discount_factor=0.9):
    # Inicialización de los valores de los nodos
    for node in graph.values():
        node.value = 0
    
    while True:
        delta = 0
        # Iterar sobre cada nodo en el grafo
        for node in graph.values():
            # Calcular el nuevo valor del nodo
            new_value = max([sum([successor.value * successor.probability for successor in node.successors]) for node in graph.values()])
            # Calcular la diferencia entre el valor nuevo y el antiguo
            delta = max(delta, abs(new_value - node.value))
            # Actualizar el valor del nodo
            node.value = new_value
        # Verificar si se ha alcanzado la convergencia
        if delta < threshold:
            break
    
    # Devolver los valores óptimos
    return {node.state: node.value for node in graph.values()}

# Ejemplo de uso
if __name__ == "__main__":
    # Definir el grafo de ejemplo
    graph = {
        "A": Node(state="A"),
        "B": Node(state="B"),
        "C": Node(state="C")
    }
    # Definir las relaciones entre los nodos
    graph["A"].successors = [graph["B"], graph["C"]]
    graph["B"].successors = [graph["C"]]
    graph["C"].successors = [graph["A"]]
    # Asignar probabilidades de transición a las relaciones
    for node in graph.values():
        for successor in node.successors:
            successor.probability = 1 / len(node.successors)
    # Ejecutar el algoritmo de Iteración de Valores
    optimal_values = value_iteration(graph)
    print("Valores óptimos de los estados:")
    for state, value in optimal_values.items():
        print(f"Estado: {state}, Valor óptimo: {value}")
