#Victor Eduardo Aleman Padilla 21310193
class FOIL:
    def __init__(self, positive_examples, negative_examples, target_predicate):
        # Inicializa la clase FOIL con ejemplos positivos, ejemplos negativos y el predicado objetivo.
        self.positive_examples = positive_examples
        self.negative_examples = negative_examples
        self.target_predicate = target_predicate

    def learn(self):
        # Inicializa la regla vacía.
        rule = []
        
        # Itera sobre los atributos y valores de los ejemplos positivos para generar la regla.
        for attr, value in self.positive_examples.items():
            if value == True:
                rule.append((attr, True))  # Añade el atributo como verdadero a la regla.
            else:
                rule.append((attr, False))  # Añade el atributo como falso a la regla.
        
        # Itera sobre los atributos y valores de los ejemplos negativos para agregar condiciones a la regla.
        for attr, value in self.negative_examples.items():
            if value == True:
                rule.append((attr, False))  # Añade el atributo como falso a la regla.
            else:
                rule.append((attr, True))  # Añade el atributo como verdadero a la regla.
        
        # Añade el predicado objetivo al final de la regla.
        rule.append((self.target_predicate, True))

        return rule  # Retorna la regla aprendida.

# Ejemplo de uso:
positive_examples = {'Sunny': True, 'Warm': True, 'High': False}
negative_examples = {'Rainy': True, 'Cool': False, 'High': True}
target_predicate = 'PlayTennis'

# Crea una instancia de la clase FOIL con ejemplos positivos, ejemplos negativos y el predicado objetivo.
foil = FOIL(positive_examples, negative_examples, target_predicate)
# Aplica el algoritmo FOIL para aprender una regla.
learned_rule = foil.learn()

print("Learned rule:", learned_rule)
