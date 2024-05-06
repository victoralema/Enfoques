#Victor Eduardo Aleman Padilla 21310193

import networkx as nx  # Importamos la librería NetworkX como 'nx' para trabajar con grafos

def cut_conditioning(graph): #Define una función llamada cut_conditioning que toma como argumento un grafo graph representado con NetworkX.
    # Creamos un diccionario para almacenar los pesos de las aristas
    edge_weights = {(u, v): graph[u][v]['weight'] for u, v in graph.edges()}
    
    # Inicializamos el corte como un conjunto vacío
    cut_set = set()
    
    # Iteramos hasta que no queden aristas por cruzar
    while len(edge_weights) > 0:
        # Seleccionamos la arista con el menor peso
        min_edge = min(edge_weights, key=edge_weights.get)
        
        # Agregamos los nodos de la arista al conjunto del corte
        cut_set.add(min_edge[0])
        cut_set.add(min_edge[1])
        
        # Eliminamos las aristas conectadas a los nodos del corte
        to_remove = []
        for edge in edge_weights:
            if min_edge[0] in edge or min_edge[1] in edge:
                to_remove.append(edge)
        
        for edge in to_remove:
            del edge_weights[edge]
    
    return cut_set #Devuelve el conjunto cut_set que contiene los nodos que forman el corte mínimo.

# Ejemplo de uso
if __name__ == "__main__":
    # Creamos un grafo no dirigido ponderado
    G = nx.Graph()
    G.add_edge('A', 'B', weight=4)
    G.add_edge('B', 'C', weight=2)
    G.add_edge('C', 'D', weight=3)
    G.add_edge('D', 'A', weight=1)
    G.add_edge('A', 'C', weight=5)

    # Aplicamos el acondicionamiento del corte al grafo
    min_cut = cut_conditioning(G)
    
    # Imprimimos el resultado del corte mínimo
    print("Conjunto del corte mínimo:", min_cut)
