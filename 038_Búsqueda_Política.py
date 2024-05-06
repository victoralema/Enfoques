#Victor Eduardo Aleman Padilla 21310193

import heapq

class Grafo:
    def __init__(self):
        self.nodos = set()# Conjunto para almacenar los nodos del grafo
        self.aristas = {}# Diccionario para almacenar las aristas del grafo
    
    def agregar_nodo(self, nodo):
        self.nodos.add(nodo)# Agregar el nodo al conjunto de nodos
        self.aristas[nodo] = []# Inicializar las aristas del nodo como una lista vacía
    
    def agregar_arista(self, origen, destino, costo):
        self.aristas[origen].append((destino, costo))# Agregar una arista desde el nodo origen al nodo destino con un costo dado
        # Para grafos no dirigidos, descomentar la siguiente línea:
        # self.aristas[destino].append((origen, costo))
    
    def obtener_vecinos(self, nodo):
        return self.aristas[nodo]# Retorna la lista de aristas (vecinos) del nodo dado
    
def buscar_camino_astar(grafo, inicio, objetivo):
    # Estructuras de datos para mantener el costo estimado y el camino
    costo_estimado = {nodo: float('inf') for nodo in grafo.nodos}# Diccionario para almacenar el costo estimado desde el nodo inicial hasta cada nodo
    costo_estimado[inicio] = 0# El costo estimado desde el nodo inicial hasta sí mismo es cero
    camino = {inicio: None}# Diccionario para almacenar el camino más corto desde el nodo inicial hasta cada nodo
    heap = [(0, inicio)]  # Min-heap de (costo acumulado + estimación, nodo)

    while heap:
        costo_actual, nodo_actual = heapq.heappop(heap)# Extraer el nodo con el menor costo acumulado + estimación desde la cola de prioridad

        if nodo_actual == objetivo:
            # Reconstruir el camino desde el objetivo hasta el inicio
            ruta = []
            paso = objetivo
            while paso is not None:
                ruta.append(paso)
                paso = camino[paso]
            return list(reversed(ruta))# Retornar la ruta encontrada (invertir la lista para obtenerla desde el inicio hasta el objetivo)

        for vecino, costo in grafo.obtener_vecinos(nodo_actual):
            nuevo_costo = costo_actual + costo
            if nuevo_costo < costo_estimado[vecino]:
                costo_estimado[vecino] = nuevo_costo
                camino[vecino] = nodo_actual
                heapq.heappush(heap, (nuevo_costo + heuristica(vecino, objetivo), vecino))
    
    return None  # No se encontró camino

def heuristica(nodo, objetivo):
    # Aquí utilizamos una heurística simple: distancia euclidiana entre nodos
    return ((nodo[0] - objetivo[0]) ** 2 + (nodo[1] - objetivo[1]) ** 2) ** 0.5

# Ejemplo de uso
grafo_ejemplo = Grafo()
grafo_ejemplo.agregar_nodo((0, 0))
grafo_ejemplo.agregar_nodo((1, 1))
grafo_ejemplo.agregar_nodo((2, 2))
grafo_ejemplo.agregar_nodo((3, 3))

grafo_ejemplo.agregar_arista((0, 0), (1, 1), 1.0)
grafo_ejemplo.agregar_arista((1, 1), (2, 2), 2.0)
grafo_ejemplo.agregar_arista((0, 0), (3, 3), 3.0)
grafo_ejemplo.agregar_arista((3, 3), (2, 2), 1.5)

inicio = (0, 0)
objetivo = (2, 2)

camino_encontrado = buscar_camino_astar(grafo_ejemplo, inicio, objetivo)
if camino_encontrado:
    print("Camino encontrado:", camino_encontrado)
else:
    print("No se encontró un camino desde", inicio, "hasta", objetivo)
