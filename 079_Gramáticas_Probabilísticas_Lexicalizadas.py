#Victor Eduardo Aleman Padilla 21310193
import random  # Importamos la biblioteca random para generar números aleatorios

class LPCFG:
    def __init__(self):
        self.productions = {}  # Diccionario para almacenar las reglas de producción
        self.terminals = {}    # Diccionario para almacenar las palabras terminales y sus probabilidades

    def add_production(self, lhs, rhs, probability):
        # Método para agregar una regla de producción con su probabilidad asociada
        if lhs not in self.productions:
            self.productions[lhs] = []  # Si el símbolo izquierdo no está en el diccionario, lo inicializamos con una lista vacía
        self.productions[lhs].append((rhs, probability))  # Añadimos la regla de producción y su probabilidad a la lista correspondiente del símbolo izquierdo

    def add_terminal(self, word, probability):
        # Método para agregar una palabra terminal con su probabilidad asociada
        self.terminals[word] = probability  # Añadimos la palabra terminal y su probabilidad al diccionario de palabras terminales

    def generate_sentence(self, start_symbol):
        # Método para generar una oración recursivamente a partir del símbolo inicial dado
        sentence = []  # Inicializamos una lista para almacenar la oración generada
        self.generate_recursive(start_symbol, sentence)  # Llamamos a un método auxiliar recursivo para generar la oración
        return ' '.join(sentence)  # Devolvemos la oración generada como una cadena de texto

    def generate_recursive(self, symbol, sentence):
        # Método auxiliar recursivo para generar la oración a partir de un símbolo dado
        if symbol in self.productions:
            # Si el símbolo tiene reglas de producción asociadas, elegimos una regla basada en las probabilidades y generamos recursivamente la oración
            productions = self.productions[symbol]  # Obtenemos las reglas de producción asociadas al símbolo
            rhs, probabilities = zip(*productions)  # Separamos las reglas de producción y sus probabilidades
            chosen_production = random.choices(rhs, weights=probabilities)[0]  # Elegimos una regla de producción basada en sus probabilidades

            for s in chosen_production:
                # Iteramos sobre los símbolos de la regla de producción elegida y generamos recursivamente la oración
                self.generate_recursive(s, sentence)
        elif symbol in self.terminals:
            # Si el símbolo es una palabra terminal, comprobamos aleatoriamente si se agrega según su probabilidad
            if random.random() < self.terminals[symbol]:
                sentence.append(symbol)  # Añadimos la palabra terminal a la oración

# Ejemplo de uso
lpcfg = LPCFG()

# Definimos algunas reglas de producción con probabilidades
lpcfg.add_production('S', ['NP', 'VP'], 1.0)
lpcfg.add_production('NP', ['Det', 'N'], 0.6)
lpcfg.add_production('NP', ['N'], 0.4)
lpcfg.add_production('VP', ['V', 'NP'], 0.7)
lpcfg.add_production('VP', ['V'], 0.3)
lpcfg.add_production('Det', ['the'], 1.0)
lpcfg.add_production('N', ['cat'], 0.5)
lpcfg.add_production('N', ['dog'], 0.5)
lpcfg.add_production('V', ['runs'], 1.0)

# Definimos algunas palabras terminales con sus probabilidades
lpcfg.add_terminal('the', 1.0)
lpcfg.add_terminal('cat', 0.7)
lpcfg.add_terminal('dog', 0.3)
lpcfg.add_terminal('runs', 1.0)

# Generamos una oración
sentence = lpcfg.generate_sentence('S')
print("Oración generada:", sentence)
