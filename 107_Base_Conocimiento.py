#Victor Eduardo Aleman Padilla 21310193
class BaseConocimiento:
    def __init__(self):
        self.reglas = []  # Inicializa una lista vacía para almacenar las reglas de la base de conocimiento

    def agregar_regla(self, antecedente, consecuente):
        self.reglas.append((antecedente, consecuente))  # Agrega una nueva regla a la lista de reglas. Cada regla es una tupla de la forma (antecedente, consecuente).

    def consultar(self, proposicion):
        # Buscar si la proposición se puede derivar de las reglas conocidas
        for antecedente, consecuente in self.reglas:
            if antecedente == proposicion:  # Si el antecedente de una regla coincide con la proposición dada
                return consecuente  # Devuelve el consecuente de esa regla
        return None  # Si no se encuentra ninguna regla que coincida con la proposición, devuelve None

# Ejemplo de uso
if __name__ == "__main__":
    # Creamos una base de conocimiento
    base_conocimiento = BaseConocimiento()

    # Agregamos algunas reglas
    base_conocimiento.agregar_regla("llueve", "hay charcos")  # Si llueve, entonces hay charcos
    base_conocimiento.agregar_regla("hay charcos", "me mojo")  # Si hay charcos, entonces me mojo

    # Consultamos la base de conocimiento
    resultado = base_conocimiento.consultar("llueve")  # Consulta si se puede derivar alguna conclusión cuando llueve
    if resultado:
        print("Si llueve, entonces", resultado)  # Imprime la conclusión derivada
    else:
        print("No se puede derivar ninguna conclusión.")  # Imprime un mensaje indicando que no se puede derivar ninguna conclusión

    resultado = base_conocimiento.consultar("nieva")  # Consulta si se puede derivar alguna conclusión cuando nieva
    if resultado:
        print("Si nieva, entonces", resultado)  # Imprime la conclusión derivada
    else:
        print("No se puede derivar ninguna conclusión.")  # Imprime un mensaje indicando que no se puede derivar ninguna conclusión
