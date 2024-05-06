#Victor Eduardo Aleman Padilla 21310193

class Grafo:
    def __init__(self, n):
        self.n = n  # Número de nodos en el grafo
        self.matriz_adyacencia = [[0] * n for _ in range(n)]  # Inicializa la matriz de adyacencia con ceros

    def agregar_arista(self, u, v, peso):
        self.matriz_adyacencia[u][v] = peso  # Agrega una arista con el peso dado entre los nodos u y v
        self.matriz_adyacencia[v][u] = peso  # Como el grafo es no dirigido, refleja el peso en la posición opuesta de la matriz

def generar_grafo(n):
    grafo = Grafo(n)  # Crea un objeto de la clase Grafo con n nodos
    peso = 1  # Inicializa el peso de la primera arista
    for i in range(n):
        for j in range(i+1, n):
            peso = (peso * 37 + 13) % 100  # Generación pseudoaleatoria de pesos entre 1 y 100 usando una fórmula de congruencia lineal
            grafo.agregar_arista(i, j, peso)  # Agrega una arista con el peso generado entre los nodos i y j
    return grafo  # Devuelve el grafo generado

def calcular_costo(camino, grafo):
    costo = 0  # Inicializa el costo total del camino
    for i in range(len(camino) - 1):  # Itera sobre cada nodo en el camino
        costo += grafo.matriz_adyacencia[camino[i]][camino[i+1]]  # Agrega el peso de la arista entre nodos consecutivos al costo
    return costo  # Devuelve el costo total del camino

def generar_vecinos(solucion_actual):
    vecinos = []  # Inicializa la lista de vecinos
    for i in range(len(solucion_actual)):  # Itera sobre cada nodo en la solución actual
        for j in range(i + 1, len(solucion_actual)):  # Itera sobre los nodos siguientes al nodo i
            vecino = solucion_actual[:]  # Copia la solución actual para generar el vecino
            vecino[i], vecino[j] = vecino[j], vecino[i]  # Intercambia los nodos en las posiciones i y j
            vecinos.append(vecino)  # Agrega el vecino a la lista de vecinos
    return vecinos  # Devuelve la lista de vecinos

def busqueda_haz_local(grafo, tam_haz, iteraciones):
    mejor_solucion = list(range(grafo.n))  # Inicializa la mejor solución como una permutación de nodos
    mejor_costo = calcular_costo(mejor_solucion, grafo)  # Calcula el costo de la mejor solución

    for _ in range(iteraciones):  # Realiza el número especificado de iteraciones
        haz = [mejor_solucion] + [list(range(grafo.n)) for _ in range(tam_haz - 1)]  # Crea un haz con la mejor solución y soluciones aleatorias
        
        for solucion in haz:  # Itera sobre cada solución en el haz
            vecinos = generar_vecinos(solucion)  # Genera los vecinos de la solución actual
            for vecino in vecinos:  # Itera sobre cada vecino
                costo_vecino = calcular_costo(vecino, grafo)  # Calcula el costo del vecino
                if costo_vecino < mejor_costo:  # Comprueba si el vecino tiene un costo mejor que la mejor solución actual
                    mejor_solucion = vecino  # Actualiza la mejor solución con el vecino
                    mejor_costo = costo_vecino  # Actualiza el mejor costo con el costo del vecino

    return mejor_solucion, mejor_costo  # Devuelve la mejor solución encontrada y su costo mínimo

if __name__ == "__main__":
    n = 5  # Número de nodos en el grafo
    grafo = generar_grafo(n)  # Genera un grafo aleatorio

    print("Grafo:")  # Imprime el grafo generado
    for fila in grafo.matriz_adyacencia:
        print(fila)

    tam_haz = 5  # Tamaño del haz de soluciones
    iteraciones = 100  # Número de iteraciones del algoritmo

    # Ejecuta el algoritmo de búsqueda de haz local para encontrar la mejor solución y su costo mínimo
    mejor_solucion, mejor_costo = busqueda_haz_local(grafo, tam_haz, iteraciones)

    print("\nMejor solución encontrada:", mejor_solucion)  # Imprime la mejor solución encontrada
    print("Costo mínimo:", mejor_costo)  # Imprime el costo mínimo de la mejor solución
