#Victor Eduardo Aleman Padilla 21310193
class Grafo:
    def __init__(self, grafo):
        self.grafo = grafo  # Inicializa el grafo con la estructura proporcionada

    def obtener_vecinos(self, nodo):
        return self.grafo[nodo]  # Devuelve los vecinos del nodo dado

def busqueda_online(grafo, inicio, destino):
    frontera = [[inicio]]  # Inicializa la frontera con una lista que contiene el nodo de inicio como único camino
    explorado = set()  # Conjunto de nodos explorados

    while frontera:  # Mientras haya caminos en la frontera
        camino_actual = frontera.pop(0)  # Toma el primer camino de la frontera
        nodo_actual = camino_actual[-1]  # Obtiene el último nodo del camino actual

        if nodo_actual == destino:  # Si el nodo actual es el destino, se ha encontrado un camino
            return camino_actual  # Devuelve el camino encontrado

        if nodo_actual not in explorado:  # Si el nodo actual no ha sido explorado
            explorado.add(nodo_actual)  # Marca el nodo como explorado

            for vecino in grafo.obtener_vecinos(nodo_actual):  # Para cada vecino del nodo actual
                if vecino not in explorado:  # Si el vecino no ha sido explorado
                    nuevo_camino = list(camino_actual)  # Crea un nuevo camino basado en el camino actual
                    nuevo_camino.append(vecino)  # Agrega el vecino al nuevo camino
                    frontera.append(nuevo_camino)  # Agrega el nuevo camino a la frontera para explorar sus vecinos

    return None  # Si no se encuentra un camino, devuelve None

if __name__ == "__main__":
    # Definición del grafo de ejemplo
    grafo = Grafo({
        'A': ['B', 'C'],
        'B': ['A', 'D', 'E'],
        'C': ['A', 'F'],
        'D': ['B'],
        'E': ['B', 'F'],
        'F': ['C', 'E']
    })

    inicio = 'A'  # Nodo de inicio
    destino = 'F'  # Nodo de destino

    # Realiza la búsqueda online desde el nodo de inicio al nodo de destino
    camino = busqueda_online(grafo, inicio, destino)

    if camino:  # Si se encuentra un camino
        print(f"Camino encontrado de {inicio} a {destino}: {camino}")  # Imprime el camino encontrado
    else:
        print(f"No se encontró un camino de {inicio} a {destino}")  # Imprime que no se encontró un camino
