#Victor Eduardo Aleman Padilla 21310193
class InferenceEngine:
    def __init__(self, rules, initial_facts):
        # Inicialización de la clase InferenceEngine con reglas y hechos iniciales
        self.rules = rules
        self.facts = initial_facts

    def forward_chain(self):
        new_facts = set()

        # Encadenamiento hacia adelante
        for rule in self.rules:
            antecedent, consequent = rule
            # Comprobar si todos los elementos del antecedente están en los hechos
            if all(fact in self.facts for fact in antecedent):
                new_facts.add(consequent)  # Agregar el consecuente si se cumplen todas las condiciones

        if new_facts:  # Si hay nuevos hechos
            self.facts |= new_facts  # Agregar los nuevos hechos al conjunto de hechos existentes
            return True, new_facts
        else:
            return False, set()  # Si no hay nuevos hechos, devolver False

    def backward_chain(self, goal):
        if goal in self.facts:
            return True  # Si el objetivo ya está en los hechos, devuelve True
        else:
            for rule in self.rules:
                antecedent, consequent = rule
                if consequent == goal:
                    # Si el objetivo coincide con el consecuente de una regla, intentar probar los antecedentes
                    if all(self.backward_chain(ante) for ante in antecedent):
                        return True  # Si todos los antecedentes se prueban, devolver True
            return False  # Si no se puede probar, devolver False

    def run_forward_chaining(self):
        # Ejecutar el encadenamiento hacia adelante hasta que no haya nuevos hechos
        while True:
            inferred, new_facts = self.forward_chain()
            if not inferred:
                break

    def run_backward_chaining(self, goal):
        # Ejecutar el encadenamiento hacia atrás para alcanzar el objetivo
        return self.backward_chain(goal)

    def get_facts(self):
        return self.facts


def main():
    # Definir reglas y hechos iniciales
    rules = [(('p'), ('q')), (('q'), ('r'))]
    initial_facts = {'p'}

    # Inicializar el motor de inferencia
    inference_engine = InferenceEngine(rules, initial_facts)

    # Realizar encadenamiento hacia adelante
    inference_engine.run_forward_chaining()

    # Imprimir los hechos deducidos
    print("Hechos deducidos:")
    print(inference_engine.get_facts())

    # Definir el objetivo para encadenamiento hacia atrás
    goal = 'r'

    # Realizar encadenamiento hacia atrás para alcanzar el objetivo
    if inference_engine.run_backward_chaining(goal):
        print(f"El objetivo '{goal}' se puede alcanzar.")
    else:
        print(f"El objetivo '{goal}' no se puede alcanzar.")


if __name__ == "__main__":
    main()
