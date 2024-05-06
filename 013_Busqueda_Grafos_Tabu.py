#Victor Eduardo Aleman Padilla 21310193

import random  # Importa el módulo random para generar números aleatorios

# Función para generar un grafo aleatorio con pesos aleatorios
def generar_grafo(n):
    grafo = [[0] * n for _ in range(n)]  # Crea una matriz de tamaño nxn inicializada con ceros
    for i in range(n):  # Itera sobre cada nodo del grafo
        for j in range(i+1, n):  # Itera sobre los nodos siguientes al nodo i
            grafo[i][j] = random.randint(1, 100)  # Asigna un peso aleatorio entre 1 y 100 a la arista (i, j)
            grafo[j][i] = grafo[i][j]  # El grafo es no dirigido, por lo que refleja el peso en la posición (j, i)
    return grafo  # Devuelve la matriz de adyacencia que representa el grafo

# Función para calcular el costo total de un camino en el grafo
def calcular_costo(camino, grafo):
    costo = 0  # Inicializa el costo total del camino
    for i in range(len(camino) - 1):  # Itera sobre cada nodo en el camino
        costo += grafo[camino[i]][camino[i+1]]  # Agrega el peso de la arista entre los nodos consecutivos al costo
    return costo  # Devuelve el costo total del camino

# Función para encontrar un vecino aleatorio para el nodo actual, evitando los nodos en la lista tabú
def encontrar_vecino(actual, lista_tabu, grafo):
    vecinos = [i for i in range(len(grafo)) if i != actual and i not in lista_tabu]  # Encuentra todos los nodos que no están en la lista tabú ni son el nodo actual
    return random.choice(vecinos)  # Devuelve un vecino aleatorio de la lista de vecinos

# Función principal que realiza la búsqueda tabú para encontrar el camino mínimo desde el nodo de inicio en el grafo dado
def busqueda_tabu(inicio, grafo, tam_tabu, iteraciones):
    actual = inicio  # Inicializa el nodo actual con el nodo de inicio
    mejor_camino = [actual]  # Inicializa el mejor camino con el nodo de inicio
    mejor_costo = float('inf')  # Inicializa el mejor costo con infinito positivo
    lista_tabu = []  # Inicializa la lista tabú vacía

    for _ in range(iteraciones):  # Itera el número especificado de veces
        vecino = encontrar_vecino(actual, lista_tabu, grafo)  # Encuentra un vecino aleatorio del nodo actual
        costo_vecino = calcular_costo([actual, vecino], grafo)  # Calcula el costo del camino entre el nodo actual y el vecino

        if costo_vecino < mejor_costo:  # Comprueba si el costo del vecino es menor que el mejor costo actual
            mejor_camino = [actual, vecino]  # Actualiza el mejor camino con el camino al vecino
            mejor_costo = costo_vecino  # Actualiza el mejor costo con el costo del vecino

        lista_tabu.append(actual)  # Agrega el nodo actual a la lista tabú
        if len(lista_tabu) > tam_tabu:  # Comprueba si la lista tabú ha alcanzado su tamaño máximo
            lista_tabu.pop(0)  # Elimina el primer elemento de la lista tabú (FIFO)

        actual = vecino  # Actualiza el nodo actual con el vecino para la siguiente iteración

    return mejor_camino, mejor_costo  # Devuelve el mejor camino encontrado y su costo mínimo

if __name__ == "__main__":  # Entrada principal del programa
    random.seed(42)  # Establece la semilla del generador de números aleatorios para reproducibilidad
    n = 5  # Número de nodos en el grafo
    grafo = generar_grafo(n)  # Genera un grafo aleatorio
    inicio = 0  # Nodo de inicio para la búsqueda tabú

    print("Grafo:")  # Imprime el grafo generado
    for fila in grafo:
        print(fila)

    tam_tabu = 2  # Tamaño de la lista tabú
    iteraciones = 10  # Número de iteraciones para la búsqueda tabú
    camino_minimo, costo_minimo = busqueda_tabu(inicio, grafo, tam_tabu, iteraciones)  # Ejecuta la búsqueda tabú

    print("\nMejor camino encontrado:", camino_minimo)  # Imprime el mejor camino encontrado
    print("Costo mínimo:", costo_minimo)  # Imprime el costo mínimo del mejor camino
