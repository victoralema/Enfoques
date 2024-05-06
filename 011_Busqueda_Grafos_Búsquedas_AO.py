#Victor Eduardo Aleman Padilla 21310193

import heapq  # Importa el módulo heapq para utilizar colas de prioridad

# Definición de la estructura del grafo
grafo = {
    'A': {'B': 5, 'C': 10},  # Nodo A con conexiones a B y C, con respectivos costos
    'B': {'D': 15, 'E': 20},  # Nodo B con conexiones a D y E, con respectivos costos
    'C': {'F': 25},           # Nodo C con conexión a F, con costo
    'D': {},                  # Nodo D sin conexiones salientes
    'E': {'F': 30},           # Nodo E con conexión a F, con costo
    'F': {}                   # Nodo F sin conexiones salientes
}

# Función de búsqueda AO*
def busqueda_ao_estrella(grafo, inicio, objetivo, alpha):
    frontera = []  # Lista de prioridad para nodos frontera (heap)
    heapq.heappush(frontera, (0, inicio, [inicio]))  # Inicialización con el nodo inicial y su costo 0
    visitados = set()  # Conjunto para mantener los nodos ya visitados durante la búsqueda

    while frontera:  # Mientras la frontera no esté vacía
        costo_actual, nodo_actual, camino_actual = heapq.heappop(frontera)  # Se extrae el nodo con menor costo de la frontera

        if nodo_actual == objetivo:
            return camino_actual  # Se encontró el objetivo, se devuelve el camino

        visitados.add(nodo_actual)  # Se marca el nodo actual como visitado

        for vecino, costo in grafo[nodo_actual].items():  # Se recorren los vecinos del nodo actual
            if vecino not in visitados:  # Si el vecino no ha sido visitado
                nuevo_camino = list(camino_actual)  # Se crea un nuevo camino basado en el camino actual
                nuevo_camino.append(vecino)         # Se añade el vecino al nuevo camino
                # Se calcula el costo total hasta el vecino considerando el factor de adaptación heurística (alpha)
                costo_total = costo_actual + costo + alpha * heuristica(vecino, objetivo)
                # Se añade el vecino a la frontera con su costo total y el nuevo camino
                heapq.heappush(frontera, (costo_total, vecino, nuevo_camino))

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
alpha = 0.5        # Factor de adaptación heurística

# Llama a la función de búsqueda AO* para encontrar un camino desde el nodo inicial hasta el nodo objetivo
camino = busqueda_ao_estrella(grafo, inicio, objetivo, alpha)

if camino:
    print("Se encontró un camino al objetivo:", camino)  # Se imprime el camino si se encuentra
else:
    print("No se encontró un camino al objetivo.")       # Se informa si no se encuentra un camino
