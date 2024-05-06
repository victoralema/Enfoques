#Victor Eduardo Aleman Padilla 21310193
class Proposition:
    def __init__(self, name, value=False):
        self.name = name  # Nombre de la proposición
        self.value = value  # Valor de verdad de la proposición (por defecto False)

class ModalOperator:
    def __init__(self, operator, proposition):
        self.operator = operator  # Operador modal (□ o ◇)
        self.proposition = proposition  # Proposición asociada al operador modal

class TemporalOperator:
    def __init__(self, operator, time):
        self.operator = operator  # Operador temporal (U, X, F, G, etc.)
        self.time = time  # Tiempo asociado al operador temporal

class FuzzySet:
    def __init__(self, name, membership_function):
        self.name = name  # Nombre del conjunto difuso
        self.membership_function = membership_function  # Función de pertenencia del conjunto difuso

# Ejemplo de uso
prop1 = Proposition("P")  # Crear una proposición llamada "P"
prop2 = Proposition("Q")  # Crear una proposición llamada "Q"
modal_op = ModalOperator("□", prop1)  # Crear un operador modal □ asociado a la proposición "P"
temporal_op = TemporalOperator("F", 5)  # Crear un operador temporal F asociado al tiempo 5

# Imprimir información sobre las estructuras creadas
print("Proposition:", prop1.name)  # Imprimir el nombre de la proposición "P"
print("Modal Operator:", modal_op.operator, modal_op.proposition.name)  # Imprimir el operador modal y el nombre de la proposición asociada
print("Temporal Operator:", temporal_op.operator, temporal_op.time)  # Imprimir el operador temporal y el tiempo asociado
