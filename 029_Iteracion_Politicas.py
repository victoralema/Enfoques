#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importamos la librería numpy para trabajar con matrices y operaciones numéricas

class Grafo:
    def __init__(self, num_nodos):
        self.num_nodos = num_nodos  # Número de nodos en el grafo
        self.matriz_adyacencia = np.zeros((num_nodos, num_nodos))  # Matriz de adyacencia del grafo
    
    def agregar_arista(self, origen, destino, peso):
        self.matriz_adyacencia[origen][destino] = peso  # Agrega una arista con el peso especificado
    
    def obtener_vecinos(self, nodo):
        return [i for i in range(self.num_nodos) if self.matriz_adyacencia[nodo][i] != 0]  # Obtiene los vecinos de un nodo
    
    def obtener_peso_arista(self, origen, destino):
        return self.matriz_adyacencia[origen][destino]  # Obtiene el peso de una arista

def iteracion_politicas(grafo, inicio, objetivo, gamma=0.8, epsilon=1e-6):
    politica = np.zeros(grafo.num_nodos)  # Inicializa la política para cada estado en 0
    politica_anterior = np.zeros(grafo.num_nodos)  # Inicializa la política anterior en 0
    
    while np.any(politica != politica_anterior):  # Itera hasta que la política converja
        politica_anterior = np.copy(politica)  # Guarda la política actual
        for estado in range(grafo.num_nodos):  # Itera sobre cada estado en el grafo
            if estado == objetivo:  # Si el estado es el objetivo, no se hace nada
                continue
            vecinos = grafo.obtener_vecinos(estado)  # Obtiene los vecinos del estado actual
            max_valor = float('-inf')  # Inicializa el valor máximo en negativo infinito
            mejor_accion = None  # Inicializa la mejor acción como None
            for accion in vecinos:  # Itera sobre cada vecino (acción)
                valor = grafo.obtener_peso_arista(estado, accion) + gamma * politica[accion]  # Calcula el valor de la acción
                if valor > max_valor:  # Si el valor es mayor que el máximo actual
                    max_valor = valor  # Actualiza el valor máximo
                    mejor_accion = accion  # Actualiza la mejor acción
            politica[estado] = max_valor  # Asigna el valor máximo a la política para el estado actual
    return politica  # Devuelve la política óptima

# Ejemplo de uso
grafo = Grafo(5)  # Crea un grafo con 5 nodos
grafo.agregar_arista(0, 1, 5)  # Agrega una arista del nodo 0 al nodo 1 con peso 5
grafo.agregar_arista(0, 2, 2)  # Agrega una arista del nodo 0 al nodo 2 con peso 2
grafo.agregar_arista(1, 3, 4)  # Agrega una arista del nodo 1 al nodo 3 con peso 4
grafo.agregar_arista(2, 1, 1)  # Agrega una arista del nodo 2 al nodo 1 con peso 1
grafo.agregar_arista(2, 3, 7)  # Agrega una arista del nodo 2 al nodo 3 con peso 7
grafo.agregar_arista(3, 4, 3)  # Agrega una arista del nodo 3 al nodo 4 con peso 3

inicio = 0  # Nodo de inicio
objetivo = 4  # Nodo objetivo

politica_optima = iteracion_politicas(grafo, inicio, objetivo)  # Encuentra la política óptima
print("Política óptima:", politica_optima)  # Imprime la política óptima
