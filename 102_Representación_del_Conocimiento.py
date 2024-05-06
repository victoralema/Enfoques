#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase KnowledgeBase para representar la base de conocimiento
class KnowledgeBase:
    # Constructor de la clase
    def __init__(self):
        # Inicialización de los hechos como un conjunto vacío
        self.facts = set()
        # Inicialización de las reglas como una lista vacía
        self.rules = []

    # Método para agregar un hecho a la base de conocimiento
    def add_fact(self, fact):
        # Agrega el hecho al conjunto de hechos
        self.facts.add(fact)

    # Método para agregar una regla a la base de conocimiento
    def add_rule(self, rule):
        # Agrega la regla a la lista de reglas
        self.rules.append(rule)

    # Método para realizar inferencias en la base de conocimiento
    def infer(self):
        # Conjunto para almacenar los hechos inferidos
        inferred_facts = set()
        # Itera sobre cada regla en la lista de reglas
        for rule in self.rules:
            # Comprueba si todos los antecedentes de la regla están presentes en los hechos conocidos
            if all(fact in self.facts for fact in rule[0]):
                # Si es así, agrega el consecuente de la regla como un nuevo hecho inferido
                inferred_facts.add(rule[1])
        # Agrega los hechos inferidos a los hechos conocidos
        self.facts |= inferred_facts

# Creación de una instancia de la base de conocimiento
kb = KnowledgeBase()

# Hechos iniciales
kb.add_fact("A")
kb.add_fact("B")

# Reglas (cada regla es una tupla de la forma (antecedente, consecuente))
kb.add_rule(({"A", "B"}, "C"))
kb.add_rule(({"C"}, "D"))

# Inferir nuevos hechos
kb.infer()

# Imprimir hechos conocidos después de la inferencia
print("Hechos conocidos después de la inferencia:")
for fact in kb.facts:
    print(fact)
