#Victor Eduardo Aleman Padilla 21310193
import random
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
    costo += grafo.matriz_adyacencia[camino[-1]][camino[0]]  # Agregar el costo de volver al inicio
    return costo  # Devuelve el costo total del camino

def generar_poblacion_inicial(grafo, tam_poblacion):
    poblacion = []  # Inicializa la población
    for _ in range(tam_poblacion):  # Repite para cada individuo en la población
        individuo = list(range(grafo.n))  # Genera un individuo como una permutación de los nodos
        poblacion.append(individuo)  # Agrega el individuo a la población
    return poblacion  # Devuelve la población generada

def seleccionar_padres(poblacion, grafo):
    padres_seleccionados = []  # Inicializa la lista de padres seleccionados
    for _ in range(len(poblacion)):  # Repite para cada individuo en la población
        torneo = poblacion[:3]  # Toma los primeros tres individuos como participantes del torneo
        torneo.sort(key=lambda x: calcular_costo(x, grafo))  # Ordena los participantes del torneo por su costo
        mejor_padre = torneo[0]  # Selecciona al individuo con el menor costo como padre
        padres_seleccionados.append(mejor_padre)  # Agrega al padre seleccionado a la lista de padres
    return padres_seleccionados  # Devuelve la lista de padres seleccionados

def cruzar_padres(padres_seleccionados):
    hijos = []  # Inicializa la lista de hijos
    for i in range(0, len(padres_seleccionados), 2):  # Itera de dos en dos sobre los padres seleccionados
        padre1 = padres_seleccionados[i]  # Selecciona el primer padre
        padre2 = padres_seleccionados[i+1]  # Selecciona el segundo padre
        punto_corte = len(padre1) // 2  # Punto de corte para el cruce
        hijo1 = padre1[:punto_corte] + [x for x in padre2 if x not in padre1[:punto_corte]]  # Genera el primer hijo
        hijo2 = padre2[:punto_corte] + [x for x in padre1 if x not in padre2[:punto_corte]]  # Genera el segundo hijo
        hijos.extend([hijo1, hijo2])  # Agrega los hijos a la lista de hijos
    return hijos  # Devuelve la lista de hijos generados

def mutar_hijos(hijos, prob_mutacion):
    for hijo in hijos:  # Itera sobre cada hijo
        if random.random() < prob_mutacion:  # Determina si se aplica una mutación al hijo
            # Realiza una mutación intercambiando dos posiciones aleatorias
            idx1, idx2 = random.sample(range(len(hijo)), 2)
            hijo[idx1], hijo[idx2] = hijo[idx2], hijo[idx1]
    return hijos  # Devuelve la lista de hijos, posiblemente mutados

def algoritmo_genetico(grafo, tam_poblacion, num_generaciones, prob_mutacion):
    poblacion = generar_poblacion_inicial(grafo, tam_poblacion)  # Genera la población inicial
    for _ in range(num_generaciones):  # Realiza un número especificado de generaciones
        padres_seleccionados = seleccionar_padres(poblacion, grafo)  # Selecciona los padres
        hijos = cruzar_padres(padres_seleccionados)  # Cruza los padres para generar hijos
        hijos_mutados = mutar_hijos(hijos, prob_mutacion)  # Aplica una mutación a los hijos
        poblacion = hijos_mutados  # Actualiza la población con los hijos mutados
    mejor_individuo = min(poblacion, key=lambda x: calcular_costo(x, grafo))  # Encuentra el mejor individuo de la población final
    mejor_costo = calcular_costo(mejor_individuo, grafo)  # Calcula el costo del mejor individuo
    return mejor_individuo, mejor_costo  # Devuelve el mejor individuo y su costo mínimo

if __name__ == "__main__":
    random.seed(42)  # Semilla para reproducibilidad
    n = 5  # Número de nodos en el grafo
    grafo = generar_grafo(n)  # Genera un grafo ponderado

    print("Grafo:")  # Imprime el grafo generado
    for fila in grafo.matriz_adyacencia:
        print(fila)

    tam_poblacion = 10  # Tamaño de la población
    num_generaciones = 100  # Número de generaciones del algoritmo
    prob_mutacion = 0.1  # Probabilidad de mutación

    mejor_solucion, mejor_costo = algoritmo_genetico(grafo, tam_poblacion, num_generaciones, prob_mutacion)

    print("\nMejor solución encontrada:", mejor_solucion)  # Imprime la mejor solución encontrada
    print("Costo mínimo:", mejor_costo)  # Imprime el costo mínimo de la mejor solución
