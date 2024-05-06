#Victor Eduardo Aleman Padilla 21310193
class FuzzyLogicAgent:
    def __init__(self):
        self.universe = {}  # Universo de discurso (conjunto de posibles valores)
        self.membership_functions = {}  # Funciones de membresía de los conjuntos difusos

    def add_universe_variable(self, variable_name, min_val, max_val):
        """
        Adds a variable to the universe of discourse.
        """
        self.universe[variable_name] = (min_val, max_val)  # Asigna un rango de valores al nombre de la variable

    def add_membership_function(self, variable_name, set_name, membership_function):
        """
        Adds a membership function to a fuzzy set.
        """
        if variable_name not in self.membership_functions:
            self.membership_functions[variable_name] = {}  # Crea un diccionario si no existe para esa variable
        self.membership_functions[variable_name][set_name] = membership_function  # Asigna la función de membresía al conjunto difuso

    def membership_degree(self, variable_name, set_name, value):
        """
        Calculates the degree of membership for a value in a fuzzy set.
        """
        if variable_name not in self.membership_functions or set_name not in self.membership_functions[variable_name]:
            return None  # Retorna None si la función de membresía no está definida
        return self.membership_functions[variable_name][set_name](value)  # Retorna el resultado de aplicar la función de membresía al valor dado

# Ejemplo de uso
if __name__ == "__main__":
    agent = FuzzyLogicAgent()

    # Definición del universo de discurso
    agent.add_universe_variable("Temperatura", 0, 100)  # Definición de la variable "Temperatura" con un rango de 0 a 100
    agent.add_universe_variable("Humedad", 0, 100)  # Definición de la variable "Humedad" con un rango de 0 a 100

    # Definición de funciones de membresía
    agent.add_membership_function("Temperatura", "Frío", lambda x: max(0, min(1, (20 - x) / 20)))  # Definición de la función de membresía para "Frío"
    agent.add_membership_function("Temperatura", "Cálido", lambda x: max(0, min(1, (x - 20) / 20)))  # Definición de la función de membresía para "Cálido"
    agent.add_membership_function("Humedad", "Seco", lambda x: max(0, min(1, (40 - x) / 40)))  # Definición de la función de membresía para "Seco"
    agent.add_membership_function("Humedad", "Húmedo", lambda x: max(0, min(1, (x - 40) / 40)))  # Definición de la función de membresía para "Húmedo"

    # Consultas sobre el grado de pertenencia
    print("Grado de pertenencia a 'Frío' para una temperatura de 15°C:", agent.membership_degree("Temperatura", "Frío", 15))
    print("Grado de pertenencia a 'Cálido' para una temperatura de 30°C:", agent.membership_degree("Temperatura", "Cálido", 30))
    print("Grado de pertenencia a 'Seco' para una humedad del 30%:", agent.membership_degree("Humedad", "Seco", 30))
    print("Grado de pertenencia a 'Húmedo' para una humedad del 60%:", agent.membership_degree("Humedad", "Húmedo", 60))
