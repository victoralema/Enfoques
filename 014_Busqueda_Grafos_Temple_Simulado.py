#Victor Eduardo Aleman Padilla 21310193

class Grafo:
    def __init__(self, n):
        self.n = n  # Número de nodos en el grafo
        self.matriz_adyacencia = [[0] * n for _ in range(n)]  # Inicializa una matriz de adyacencia de tamaño nxn con ceros

def generar_grafo(n):
    grafo = Grafo(n)  # Crea un objeto Grafo con n nodos
    for i in range(n):  # Itera sobre los nodos del grafo
        for j in range(i+1, n):  # Itera sobre los nodos siguientes al nodo i
            # Generación pseudoaleatoria de pesos entre 1 y 100 para las aristas
            peso = (17 * i + 29 * j + 11) % 100 + 1
            grafo.matriz_adyacencia[i][j] = peso  # Asigna el peso a la arista entre los nodos i y j
            grafo.matriz_adyacencia[j][i] = peso  # Como el grafo es no dirigido, refleja el peso en la posición opuesta de la matriz
    return grafo  # Devuelve el grafo generado

def calcular_costo(camino, grafo):
    costo = 0  # Inicializa el costo total del camino
    for i in range(len(camino) - 1):  # Itera sobre cada nodo en el camino
        costo += grafo.matriz_adyacencia[camino[i]][camino[i+1]]  # Suma el peso de la arista entre nodos consecutivos al costo
    return costo  # Devuelve el costo total del camino

def generar_vecino(solucion_actual):
    vecino = solucion_actual[:]  # Copia la solución actual para generar el vecino
    # Generación pseudoaleatoria de índices para intercambiar nodos en la solución
    i = (17 * (len(vecino) // 2) + 29 * (len(vecino) // 3) + 11) % len(vecino)
    j = (23 * (len(vecino) // 5) + 31 * (len(vecino) // 7) + 13) % len(vecino)
    vecino[i], vecino[j] = vecino[j], vecino[i]  # Intercambia los nodos en las posiciones i y j
    return vecino  # Devuelve el vecino generado

def temple_simulado(grafo, temperatura_inicial, factor_enfriamiento, iteraciones):
    solucion_actual = list(range(grafo.n))  # Inicializa la solución actual como una permutación de nodos
    costo_actual = calcular_costo(solucion_actual, grafo)  # Calcula el costo de la solución actual
    mejor_solucion = solucion_actual[:]  # Inicializa la mejor solución con la solución actual
    mejor_costo = costo_actual  # Inicializa el mejor costo con el costo de la solución actual

    temperatura = temperatura_inicial  # Inicializa la temperatura

    for _ in range(iteraciones):  # Realiza el número especificado de iteraciones
        vecino = generar_vecino(solucion_actual)  # Genera un vecino de la solución actual
        costo_vecino = calcular_costo(vecino, grafo)  # Calcula el costo del vecino

        if costo_vecino < costo_actual:  # Comprueba si el vecino tiene un costo mejor que el actual
            solucion_actual = vecino  # Actualiza la solución actual con el vecino
            costo_actual = costo_vecino  # Actualiza el costo actual con el costo del vecino
            if costo_vecino < mejor_costo:  # Comprueba si el vecino tiene el mejor costo encontrado hasta ahora
                mejor_solucion = vecino  # Actualiza la mejor solución con el vecino
                mejor_costo = costo_vecino  # Actualiza el mejor costo con el costo del vecino
        else:
            # Simula la aceptación del vecino con una probabilidad basada en la diferencia de costos y la temperatura
            if (17 * costo_vecino + 29 * costo_actual + 11) % 100 < temperatura:
                solucion_actual = vecino  # Actualiza la solución actual con el vecino
                costo_actual = costo_vecino  # Actualiza el costo actual con el costo del vecino

        temperatura *= factor_enfriamiento  # Reduce la temperatura en cada iteración

    return mejor_solucion, mejor_costo  # Devuelve la mejor solución encontrada y su costo mínimo

if __name__ == "__main__":
    n = 5  # Número de nodos en el grafo
    grafo = generar_grafo(n)  # Genera un grafo aleatorio

    print("Grafo:")  # Imprime el grafo generado
    for fila in grafo.matriz_adyacencia:
        print(fila)

    temperatura_inicial = 100  # Temperatura inicial para el temple simulado
    factor_enfriamiento = 0.95  # Factor de enfriamiento para reducir la temperatura en cada iteración
    iteraciones = 1000  # Número de iteraciones del algoritmo

    # Ejecuta el algoritmo de temple simulado para encontrar la mejor solución y su costo
    mejor_solucion, mejor_costo = temple_simulado(grafo, temperatura_inicial, factor_enfriamiento, iteraciones)

    print("\nMejor solución encontrada:", mejor_solucion)  # Imprime la mejor solución encontrada
    print("Costo mínimo:", mejor_costo)  # Imprime el costo mínimo de la mejor solución
