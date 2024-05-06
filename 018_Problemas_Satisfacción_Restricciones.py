#Victor Eduardo Aleman Padilla 21310193

class MapColoringCSP: #Define una clase llamada MapColoringCSP que representa un problema de CSP para el problema de coloración de mapas.
    def __init__(self, variables, domains, neighbors): #Método de inicialización de la clase MapColoringCSP
        self.variables = variables  # Lista de variables (regiones)
        self.domains = domains  # Diccionario de dominios {variable: [lista de colores]}
        self.neighbors = neighbors  # Diccionario de vecinos {variable: [lista de vecinos]}

    def is_consistent(self, variable, assignment, color): #Define un método que verifica si asignar un color a una variable específica es consistente con la asignación actual.
        """ Verifica si asignar 'color' a 'variable' es consistente con 'assignment' actual """
        for neighbor in self.neighbors[variable]:  #Itera sobre los vecinos de la variable dada para verificar restricciones de adyacencia.
            if neighbor in assignment and assignment[neighbor] == color: #Verifica si el vecino está asignado y tiene el mismo color que el que se está intentando asignar.
                return False
        return True

    def backtracking_search(self, assignment={}): #Define el método principal para realizar la búsqueda por backtracking para resolver el CSP.
        """ Algoritmo de búsqueda mediante backtracking """
        if len(assignment) == len(self.variables): #Verifica si se ha asignado un color a todas las variables, en cuyo caso se ha encontrado una solución completa.
            return assignment  # ¡Se encontró una solución completa!

        unassigned = [var for var in self.variables if var not in assignment]  #Encuentra las variables que aún no han sido asignadas.
        first_unassigned = unassigned[0]
 
        for color in self.domains[first_unassigned]: #Itera sobre los colores disponibles para la primera variable no asignada.
            if self.is_consistent(first_unassigned, assignment, color): #Verifica si asignar ese color a la variable es consistente con la asignación actual.
                assignment[first_unassigned] = color #Asigna el color a la variable en la asignación actual.
                result = self.backtracking_search(assignment) #Llama recursivamente al método backtracking_search con la nueva asignación.
                if result is not None:
                    return result
                del assignment[first_unassigned] #Deshace la asignación si no conduce a una solución válida.

        return None  # No se encontró solución

# Ejemplo de uso
if __name__ == "__main__": #Sección principal del programa que se ejecuta si este script se ejecuta como el programa principal.
    # Definir variables, dominios y vecinos
    variables = ['WA', 'NT', 'SA', 'Q', 'NSW', 'V', 'T']
    domains = {
        'WA': ['red', 'green', 'blue'],
        'NT': ['red', 'green', 'blue'],
        'SA': ['red', 'green', 'blue'],
        'Q': ['red', 'green', 'blue'],
        'NSW': ['red', 'green', 'blue'],
        'V': ['red', 'green', 'blue'],
        'T': ['red', 'green', 'blue']
    }
    neighbors = {
        'WA': ['NT', 'SA'],
        'NT': ['WA', 'SA', 'Q'],
        'SA': ['WA', 'NT', 'Q', 'NSW', 'V'],
        'Q': ['NT', 'SA', 'NSW'],
        'NSW': ['Q', 'SA', 'V'],
        'V': ['SA', 'NSW', 'T'],
        'T': ['V']
    }

    # Crear instancia del CSP
    map_csp = MapColoringCSP(variables, domains, neighbors)

    # Resolver el CSP utilizando búsqueda por backtracking
    solution = map_csp.backtracking_search()

    if solution is not None:
        print("Solución encontrada:")
        for var, color in solution.items(): #Itera sobre la solución encontrada e imprime las asignaciones de colores para cada región.
            print(f"{var}: {color}")
    else:
        print("No se encontró solución.") #Imprime un mensaje si no se encontró una solución válida para el CSP.
