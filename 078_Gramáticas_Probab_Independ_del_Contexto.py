#Victor Eduardo Aleman Padilla 21310193
import random  # Importamos la biblioteca random para generar números aleatorios

class PCFG:
    def __init__(self):
        self.productions = {}  # Inicializamos un diccionario para almacenar las reglas de producción

    def add_production(self, lhs, rhs, probability):
        # Método para agregar una regla de producción con su probabilidad asociada
        if lhs not in self.productions:
            self.productions[lhs] = []  # Si el símbolo izquierdo no está en el diccionario, lo inicializamos con una lista vacía
        self.productions[lhs].append((rhs, probability))  # Añadimos la regla de producción y su probabilidad a la lista correspondiente del símbolo izquierdo

    def generate_sentence(self, start_symbol):
        # Método para generar una oración recursivamente a partir del símbolo inicial dado
        sentence = []  # Inicializamos una lista para almacenar la oración generada
        self.generate_recursive(start_symbol, sentence)  # Llamamos a un método auxiliar recursivo para generar la oración
        return ' '.join(sentence)  # Devolvemos la oración generada como una cadena de texto

    def generate_recursive(self, symbol, sentence):
        # Método auxiliar recursivo para generar la oración a partir de un símbolo dado
        if symbol not in self.productions:
            # Si el símbolo no tiene reglas de producción asociadas, lo agregamos directamente a la oración
            sentence.append(symbol)
            return

        productions = self.productions[symbol]  # Obtenemos las reglas de producción asociadas al símbolo
        rhs, probabilities = zip(*productions)  # Separamos las reglas de producción y sus probabilidades
        chosen_production = random.choices(rhs, weights=probabilities)[0]  # Elegimos una regla de producción basada en sus probabilidades

        for s in chosen_production:
            # Iteramos sobre los símbolos de la regla de producción elegida y generamos recursivamente la oración
            self.generate_recursive(s, sentence)

# Ejemplo de uso
pcfg = PCFG()

# Definimos algunas reglas de producción con probabilidades
pcfg.add_production('S', ['NP', 'VP'], 1.0)
pcfg.add_production('NP', ['Det', 'N'], 0.6)
pcfg.add_production('NP', ['N'], 0.4)
pcfg.add_production('VP', ['V', 'NP'], 0.7)
pcfg.add_production('VP', ['V'], 0.3)
pcfg.add_production('Det', ['el'], 1.0)
pcfg.add_production('N', ['gato'], 0.5)
pcfg.add_production('N', ['perro'], 0.5)
pcfg.add_production('V', ['corre'], 1.0)

# Generamos una oración
sentence = pcfg.generate_sentence('S')
print("Oración generada:", sentence)
