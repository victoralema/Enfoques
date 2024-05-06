#Victor Eduardo Aleman Padilla 21310193
import heapq  # Importa el módulo heapq que proporciona una implementación de la cola de prioridad

class Nodo:
    import heapq

class Nodo:
    def __init__(self, estado, padre=None, g=0, h=0):
        self.estado = estado
        self.padre = padre
        self.g = g
        self.h = h

    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()  # Menor que

    def __eq__(self, other):
        return self.estado == other.estado  # Igual a

class Problema:
    def __init__(self, inicio, objetivo, matriz):
        self.inicio = inicio  # Posición inicial del problema
        self.objetivo = objetivo  # Posición objetivo del problema
        self.matriz = matriz  # Matriz que representa el entorno con obstáculos

    def es_estado_valido(self, estado):
        x, y = estado
        if 0 <= x < len(self.matriz) and 0 <= y < len(self.matriz[0]):
            return self.matriz[x][y] == 0  # Verifica si el estado está dentro de la cuadrícula y no es un obstáculo
        return False

    def obtener_vecinos(self, estado):
        movimientos = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Movimientos: derecha, izquierda, abajo, arriba
        vecinos = [(estado[0] + dx, estado[1] + dy) for dx, dy in movimientos]  # Calcula los vecinos posibles
        return [vecino for vecino in vecinos if self.es_estado_valido(vecino)]  # Filtra los vecinos válidos

    def calcular_heuristica(self, estado):
        return abs(estado[0] - self.objetivo[0]) + abs(estado[1] - self.objetivo[1])  # Heurística de distancia de Manhattan

    def resolver(self):
        inicio = Nodo(self.inicio, g=0, h=self.calcular_heuristica(self.inicio))  # Crea el nodo inicial
        frontera = [(inicio.f(), inicio)]  # Frontera ordenada por f()
        explorado = set()  # Conjunto de estados explorados

        while frontera:  # Mientras haya nodos en la frontera
            _, nodo_actual = heapq.heappop(frontera)  # Extrae el nodo con el menor valor f de la frontera

            if nodo_actual.estado == self.objetivo:  # Si se alcanza el objetivo
                # Reconstruye el camino
                camino = []
                while nodo_actual:
                    camino.append(nodo_actual.estado)  # Agrega el estado del nodo al camino
                    nodo_actual = nodo_actual.padre  # Retrocede al nodo padre
                return list(reversed(camino))  # Devuelve el camino invertido (desde el inicio hasta el objetivo)

            explorado.add(nodo_actual.estado)  # Marca el estado actual como explorado

            for vecino_estado in self.obtener_vecinos(nodo_actual.estado):  # Para cada vecino del nodo actual
                if vecino_estado not in explorado:  # Si el vecino no ha sido explorado
                    vecino = Nodo(vecino_estado, padre=nodo_actual,  # Crea un nuevo nodo para el vecino
                                  g=nodo_actual.g + 1,
                                  h=self.calcular_heuristica(vecino_estado))
                    heapq.heappush(frontera, (vecino.f(), vecino))  # Agrega el vecino a la frontera

        return None  # Si no se encuentra solución, devuelve None

# Ejemplo de uso
inicio = (0, 0)
objetivo = (4, 4)
matriz = [
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

problema = Problema(inicio, objetivo, matriz)  # Crea una instancia del problema
ruta = problema.resolver()  # Resuelve el problema para encontrar la ruta óptima

if ruta:  # Si se encuentra una ruta
    print("Ruta encontrada:")
    for paso, estado in enumerate(ruta):
        print(f"Paso {paso}: {estado}")  # Imprime cada paso de la ruta
else:
    print("No se encontró una ruta válida.")  # Imprime si no se encuentra una ruta
