#Victor Eduardo Aleman Padilla 21310193

from collections import deque #Importa la clase deque de la biblioteca collections para utilizar una cola en la implementación del algoritmo.

class Graph: #Define una clase Graph para representar un grafo.
    def __init__(self): #Método de inicialización de la clase Graph que inicializa un diccionario vacío (self.graph) para almacenar las conexiones del grafo.
        self.graph = {}  # Inicializa un diccionario vacío para representar el grafo

    def add_edge(self, node1, node2): #Método para agregar una conexión (arista) entre dos nodos (node1 y node2) al grafo.
        if node1 not in self.graph: #Verifica si el nodo node1 no está en el grafo y, de ser así, inicializa una lista vacía para sus vecinos.
            self.graph[node1] = []  # Crea una lista vacía para los vecinos de node1 si node1 no está en el grafo
        if node2 not in self.graph: #Verifica lo mismo para el nodo node2.
            self.graph[node2] = []  # Crea una lista vacía para los vecinos de node2 si node2 no está en el grafo
        self.graph[node1].append(node2)  # Agrega node2 a la lista de vecinos de node1
        self.graph[node2].append(node1)  # Agrega node1 a la lista de vecinos de node2 (grafo no dirigido)

    def ac3(self, start_node, goal_node): #Método que implementa el algoritmo AC-3 para la propagación de restricciones.
        queue = deque([(start_node, neighbor) for neighbor in self.graph[start_node]]) #Inicializa una cola queue con pares (start_node, neighbor) para todos los vecinos de start_node.

        while queue: # Inicia un bucle mientras la cola no esté vacía.
            current_node, neighbor = queue.popleft() #Desencola un par (current_node, neighbor) de la cola.

            if self.revise(current_node, neighbor): #Llama al método revise para aplicar la revisión de restricciones entre current_node y neighbor.
                if not self.graph[current_node]:  # No hay más vecinos, no se puede continuar
                    return False
                
                for connected_node in self.graph[current_node]: # Itera sobre todos los nodos conectados a current_node.
                    if connected_node != neighbor: #Verifica si el connected_node no es igual a neighbor para evitar agregar la misma relación.
                        queue.append((connected_node, current_node)) #Encola un par (connected_node, current_node) para procesar las restricciones con el connected_node.

        return True #Devuelve True si se logra la consistencia de arco (arc consistency) en todo el grafo.

    def revise(self, current_node, neighbor): #Método que revisa y actualiza las restricciones entre current_node y neighbor.
        revised = False #Inicializa una bandera para verificar si se realizó alguna revisión.
        to_remove = [] #Inicializa una lista para almacenar los valores a eliminar de current_node.

        for value in self.graph[current_node]: #Itera sobre los valores en el dominio de current_node.
            if all(value != other_value for other_value in self.graph[neighbor]): #Verifica si value no es compatible con ningún other_value en el dominio de neighbor.
                to_remove.append(value) #Agrega value a la lista to_remove si no es compatible con ningún valor en neighbor.
                revised = True

        for value in to_remove: #Itera sobre los valores a eliminar.
            self.graph[current_node].remove(value) #Elimina value del dominio de current_node.

        return revised

    def path_exists(self, start_node, goal_node): #Método que verifica si existe un camino desde start_node hasta goal_node con restricciones propagadas.
        return self.ac3(start_node, goal_node) and goal_node in self.graph[start_node] #Devuelve True si se logra la consistencia en el grafo y goal_node está en el dominio de start_node.

# Ejemplo de uso
if __name__ == "__main__":
    graph = Graph()

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

    # Verificar si existe un camino desde el nodo inicial hasta el nodo objetivo
    if graph.path_exists(start_node, goal_node):
        print("Se encontró un camino desde {} hasta {}.".format(start_node, goal_node))
    else:
        print("No se encontró un camino desde {} hasta {}.".format(start_node, goal_node))
