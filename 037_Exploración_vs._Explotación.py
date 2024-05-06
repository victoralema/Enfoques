#Victor Eduardo Aleman Padilla 21310193

class Grafo:
    def __init__(self):
        self.graph = {}  # Inicializa un diccionario para representar el grafo

    def agregar_arista(self, u, v):
        if u in self.graph:
            self.graph[u].append(v)  # Agrega v como un vecino de u si u ya está en el grafo
        else:
            self.graph[u] = [v]  # Crea una nueva entrada en el diccionario para u con v como su vecino inicial

    def bfs(self, inicio):
        visitados = set()  # Conjunto para mantener un registro de nodos visitados
        cola = [inicio]  # Inicializa la cola con el nodo de inicio
        visitados.add(inicio)  # Marca el nodo de inicio como visitado

        while cola:
            nodo_actual = cola.pop(0)  # Extrae el primer nodo de la cola
            print(nodo_actual, end=' ')  # Imprime el nodo actual

            if nodo_actual in self.graph:
                for vecino in self.graph[nodo_actual]:  # Itera sobre los vecinos del nodo actual
                    if vecino not in visitados:  # Si el vecino no ha sido visitado
                        cola.append(vecino)  # Agrega el vecino a la cola
                        visitados.add(vecino)  # Marca el vecino como visitado

# Ejemplo de uso
grafo = Grafo()  # Crea una instancia de la clase Grafo

# Agrega aristas al grafo
grafo.agregar_arista(0, 1)
grafo.agregar_arista(0, 2)
grafo.agregar_arista(1, 2)
grafo.agregar_arista(2, 0)
grafo.agregar_arista(2, 3)
grafo.agregar_arista(3, 3)

# Realiza la búsqueda en anchura (BFS) empezando desde el nodo 2
print("Recorrido BFS empezando desde el nodo 2:")
grafo.bfs(2)
