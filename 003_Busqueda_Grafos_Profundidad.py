def dfs(graph, start, visited=None):   #Esta línea define una función llamada dfs que toma tres parámetros: graph, start  y visited .
    if visited is None:  # erifica si el parámetro visited es None, lo cual ocurre solo en la primera llamada a la función.
        visited = set()  # Conjunto para almacenar nodos visitados
    
    print(start)  # Imprimir el nodo actual al visitarlo
    visited.add(start)  # Marcar el nodo como visitado
    
    for neighbor in graph[start]:  #Itera sobre cada nodo vecino (neighbor) del nodo actual (start) en el grafo, utilizando el diccionario de listas de adyacencia (graph) para obtener los nodos vecinos.
        if neighbor not in visited:  #Verifica si el vecino (neighbor) no ha sido visitado previamente.
            dfs(graph, neighbor, visited)  # Llamada recursiva a DFS para el vecino no visitado

# Ejemplo de uso
if __name__ == "__main__": #se utiliza para asegurarse de que el código dentro de este bloque solo se ejecute cuando el script se ejecute como programa principal, no cuando se importe como un módulo.
    # Definir un grafo como un diccionario de listas de adyacencia
    graph = {
        'A': ['B', 'C'],
        'B': ['A', 'D', 'E'],
        'C': ['A', 'F'],
        'D': ['B'],
        'E': ['B', 'F'],
        'F': ['C', 'E']
    }

    start_node = 'A' #Define el nodo inicial (start_node) desde donde comenzará el recorrido DFS.

    print("Recorrido DFS comenzando desde el nodo 'A':")  # Imprime un mensaje indicando que se iniciará el recorrido DFS desde el nodo 'A'.
    dfs(graph, start_node)  # Llama a la función dfs con el grafo (graph) y el nodo inicial (start_node) para iniciar el recorrido DFS desde el nodo 'A'.
