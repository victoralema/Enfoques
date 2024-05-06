#Victor Eduardo Aleman Padilla 21310193

class KnowledgeBase:
    def __init__(self):
        # Inicializa la base de conocimientos con algunos hechos.
        self.facts = {
            "Socrates": "Mortal",
            "Plato": "Philosopher",
            "Lion": "Animal",
            "Cat": "Animal"
        }
        # Inicializa algunas reglas de inferencia.
        self.rules = {
            "Mortal": ["Human"],
            "Philosopher": ["Human"],
            "Animal": ["Quadruped"]
        }

    def explain(self, item):
        # Explica por qué un elemento es lo que es, basado en reglas de inferencia y hechos conocidos.
        explanation = f"{item} is "
        if item in self.facts:  # Si el elemento está presente en los hechos conocidos.
            explanation += self.facts[item]  # Agrega la descripción directamente.
        else:
            # Si el elemento no está presente en los hechos, busca en las reglas de inferencia.
            for fact, value in self.facts.items():  # Recorre todos los hechos conocidos.
                if item in self.rules.get(value, []):  # Si el elemento está en las reglas de inferencia.
                    explanation += f"{value} because {fact} is {value}."  # Construye la explicación.
                    break  # Termina el bucle al encontrar la primera explicación válida.
            else:
                explanation += "unknown."  # Si no se encontró una explicación, marca como desconocido.
        return explanation  # Retorna la explicación.

    def relevant_info(self, item):
        # Encuentra información relevante para un elemento dado.
        relevant_info = []  # Inicializa la lista de información relevante.
        if item in self.facts:  # Si el elemento está presente en los hechos conocidos.
            for fact, value in self.facts.items():  # Recorre todos los hechos conocidos.
                if value == self.facts[item]:  # Si comparte el mismo valor con el elemento dado.
                    relevant_info.append(fact)  # Agrega el hecho a la lista de información relevante.
        else:
            for fact, value in self.facts.items():  # Si no está presente en los hechos conocidos.
                if item in self.rules.get(value, []):  # Si el elemento está en las reglas de inferencia.
                    relevant_info.append(fact)  # Agrega el hecho a la lista de información relevante.
        return relevant_info  # Retorna la lista de información relevante.

# Ejemplo de uso:
kb = KnowledgeBase()

# Explicación sobre Socrates
explanation_socrates = kb.explain("Socrates")
print("Explanation for Socrates:", explanation_socrates)

# Información relevante sobre Animal
relevant_info_animal = kb.relevant_info("Animal")
print("Relevant information for Animal:", relevant_info_animal)
