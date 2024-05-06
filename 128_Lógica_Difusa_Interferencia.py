#Victor Eduardo Aleman Padilla 21310193

class FuzzyInferenceEngine:
    def __init__(self):
        self.rules = []  # Lista para almacenar las reglas difusas

    def add_rule(self, antecedents, consequent):
        """
        Adds a fuzzy rule to the inference engine.
        """
        self.rules.append((antecedents, consequent))  # Agrega una regla difusa a la lista de reglas

    def infer(self, inputs):
        """
        Performs fuzzy inference given inputs and returns the consequent.
        """
        outputs = {}  # Diccionario para almacenar los resultados de la inferencia
        for antecedents, consequent in self.rules:  # Itera sobre cada regla
            min_membership = min([inputs[var] if var in inputs else 0 for var in antecedents])  # Calcula el grado de pertenencia mínimo para los antecedentes difusos
            if consequent not in outputs or min_membership > outputs[consequent]:  # Si el consecuente no está en los resultados o el grado de pertenencia mínimo es mayor que el existente
                outputs[consequent] = min_membership  # Actualiza el grado de pertenencia mínimo para el consecuente
        return outputs  # Retorna los resultados de la inferencia

# Ejemplo de uso
if __name__ == "__main__":
    engine = FuzzyInferenceEngine()

    # Agregando reglas
    engine.add_rule(["Temperatura: Frío", "Humedad: Seco"], "Comodidad: Buena")
    engine.add_rule(["Temperatura: Cálido", "Humedad: Húmedo"], "Comodidad: Regular")
    engine.add_rule(["Temperatura: Cálido", "Humedad: Seco"], "Comodidad: Buena")

    # Entradas del usuario
    inputs = {
        "Temperatura: Frío": 0.7,
        "Humedad: Seco": 0.5
    }

    # Inferencia
    outputs = engine.infer(inputs)

    # Resultados
    print("Resultados de la inferencia:")
    for consequent, degree in outputs.items():
        print(f"{consequent}: {degree}")
