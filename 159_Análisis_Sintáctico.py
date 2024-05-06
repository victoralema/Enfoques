#Victor Eduardo Aleman Padilla 21310193
class Parser:
    def __init__(self, sentence):
        # Constructor de la clase Parser, inicializa la instancia con una oración dada.
        self.sentence = sentence.split()  # Divide la oración en palabras y las guarda en una lista.
        self.current_index = 0  # Inicializa el índice actual para seguir el progreso en la lista de palabras.

    def match(self, expected_token):
        # Método para comparar el siguiente token esperado con la siguiente palabra en la oración.
        if self.current_index < len(self.sentence) and self.sentence[self.current_index] == expected_token:
            # Comprueba si el índice actual está dentro del rango de la lista de palabras y si la palabra actual coincide con la esperada.
            self.current_index += 1  # Incrementa el índice para avanzar a la siguiente palabra.
        else:
            # Si no hay coincidencia, lanza una excepción de sintaxis.
            raise SyntaxError(f"Expected '{expected_token}' but found '{self.sentence[self.current_index]}'")

    def noun_phrase(self):
        # Método para analizar la frase nominal.
        self.match("the")  # Comprueba si la próxima palabra es "the".
        self.match("cat")  # Comprueba si la próxima palabra es "cat".

    def verb_phrase(self):
        # Método para analizar la frase verbal.
        self.match("sat")  # Comprueba si la próxima palabra es "sat".

    def parse_sentence(self):
        # Método para analizar la oración completa.
        self.noun_phrase()  # Analiza la frase nominal.
        self.verb_phrase()  # Analiza la frase verbal.

def main():
    sentence = "the cat sat"  # Define la oración a analizar.
    parser = Parser(sentence)  # Crea una instancia de la clase Parser con la oración.
    try:
        parser.parse_sentence()  # Intenta analizar la oración.
        print("Parsing successful! Sentence follows the grammar rules.")  # Si no hay excepciones, la oración sigue las reglas gramaticales.
    except SyntaxError as e:
        print("Parsing failed:", e)  # Si hay una excepción, la oración no sigue las reglas gramaticales.

if __name__ == "__main__":
    main()  # Llama a la función main cuando se ejecuta el script directamente.
