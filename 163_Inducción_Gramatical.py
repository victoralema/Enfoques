#Victor Eduardo Aleman Padilla 21310193
class GrammarInducer:
    def __init__(self):
        # Constructor de la clase GrammarInducer, inicializa el diccionario de reglas gramaticales.
        self.grammar_rules = {}

    def induct_grammar(self, examples):
        # Método para inducir la gramática a partir de ejemplos.
        for example in examples:
            # Iteramos sobre cada ejemplo en la lista de ejemplos.
            # Un ejemplo consiste en una lista de palabras y una lista de etiquetas gramaticales asociadas.
            words, tags = example
            for word, tag in zip(words, tags):
                # Iteramos sobre cada palabra y etiqueta gramatical en el ejemplo.
                if tag not in self.grammar_rules:
                    # Si la etiqueta no está en el diccionario de reglas gramaticales, la agregamos.
                    self.grammar_rules[tag] = set()  # Creamos un conjunto para almacenar las palabras asociadas con la etiqueta.
                self.grammar_rules[tag].add(word)  # Agregamos la palabra al conjunto asociado con la etiqueta.

    def generate_sentence(self, tag):
        # Método para generar una oración a partir de una etiqueta gramatical.
        if tag in self.grammar_rules:
            # Si la etiqueta está en el diccionario de reglas gramaticales.
            return " ".join(self.grammar_rules[tag])  # Generamos una oración combinando todas las palabras asociadas con la etiqueta.
        else:
            # Si la etiqueta no está en el diccionario, indicamos que no se encontraron reglas gramaticales para esa etiqueta.
            return "No se encontraron reglas gramaticales para la etiqueta proporcionada."

def main():
    # Función principal del programa.
    inducer = GrammarInducer()  # Creamos una instancia de la clase GrammarInducer.

    # Ejemplos de palabras con sus etiquetas gramaticales.
    examples = [
        (["The", "cat", "sat"], ["Determiner", "Noun", "Verb"]),
        (["A", "dog", "barked"], ["Determiner", "Noun", "Verb"])
    ]

    # Inducimos la gramática a partir de los ejemplos.
    inducer.induct_grammar(examples)

    # Generamos una oración utilizando la gramática inducida.
    generated_sentence = inducer.generate_sentence("Noun")
    print("Oración generada:", generated_sentence)

if __name__ == "__main__":
    main()
