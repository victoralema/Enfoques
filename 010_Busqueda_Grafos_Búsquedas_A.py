#Victor Eduardo Aleman Padilla 21310193

import heapq

def busqueda_a_estrella(grafo, inicio, objetivo):
    frontera = []  # Lista de prioridad para nodos frontera
    heapq.heappush(frontera, (0, inicio, [inicio]))  # Inicialización con el nodo inicial y su costo 0
    visitados = set()  # Conjunto para mantener los nodos ya visitados

    while frontera:
        costo_actual, nodo_actual, camino_actual = heapq.heappop(frontera)  # Se extrae el nodo con menor costo de la frontera

        if nodo_actual == objetivo:
            return camino_actual  # Se encontró el objetivo, se devuelve el camino

        visitados.add(nodo_actual)  # Se marca el nodo actual como visitado

        for vecino, costo in grafo[nodo_actual].items():  # Se recorren los vecinos del nodo actual
            if vecino not in visitados:  # Si el vecino no ha sido visitado
                nuevo_camino = list(camino_actual)  # Se crea un nuevo camino basado en el camino actual
                nuevo_camino.append(vecino)         # Se añade el vecino al nuevo camino
                costo_total = costo_actual + costo + heuristica(vecino, objetivo)  # Se calcula el costo total hasta el vecino
                heapq.heappush(frontera, (costo_total, vecino, nuevo_camino))      # Se añade el vecino a la frontera con su costo total

    return None  # No se encontró el objetivo

# Función de heurística (distancia euclidiana)
def heuristica(nodo_actual, nodo_objetivo):
    coordenadas = {  # Coordenadas de los nodos en un sistema de coordenadas ficticio
        'A': (0, 0),
        'B': (1, 0),
        'C': (0, 1),
        'D': (2, 0),
        'E': (1, 1),
        'F': (2, 1)
    }
    x1, y1 = coordenadas[nodo_actual]    # Coordenadas del nodo actual
    x2, y2 = coordenadas[nodo_objetivo]  # Coordenadas del nodo objetivo
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5  # Distancia euclidiana entre los nodos

# Ejemplo de uso
inicio = 'A'       # Nodo inicial
objetivo = 'F'     # Nodo objetivo

# Grafo representado como un diccionario de diccionarios
grafo = {
    'A': {'B': 3, 'C': 5},
    'B': {'A': 3, 'C': 2, 'D': 6},
    'C': {'A': 5, 'B': 2, 'D': 1, 'E': 8},
    'D': {'B': 6, 'C': 1, 'E': 4},
    'E': {'C': 8, 'D': 4, 'F': 2},
    'F': {'E': 2}
}

camino = busqueda_a_estrella(grafo, inicio, objetivo)  # Se realiza la búsqueda de A* para encontrar el camino
if camino:
    print("Se encontró un camino al objetivo:", camino)  # Se imprime el camino si se encuentra
else:
    print("No se encontró un camino al objetivo.")       # Se informa si no se encuentra un camino
