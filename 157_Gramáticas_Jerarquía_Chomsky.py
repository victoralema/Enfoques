#Victor Eduardo Aleman Padilla 21310193
class Tipo0Recognizer:
    def __init__(self, grammar):
        self.grammar = grammar  # Inicializa el reconocedor con la gramática proporcionada

    def recognize(self, input_string):
        # Implementar el algoritmo de reconocimiento aquí
        pass  # Aquí se implementará la lógica de reconocimiento, pero aún no está definida

# Ejemplo de uso
if __name__ == "__main__":
    # Definir una gramática de tipo 0
    tipo_0_grammar = {
        "S": ["AB"],  # Regla de producción para el símbolo S
        "A": ["aA", "a"],  # Reglas de producción para el símbolo A
        "B": ["bB", "b", ""]  # Reglas de producción para el símbolo B
    }

    recognizer = Tipo0Recognizer(tipo_0_grammar)  # Crea un objeto reconocedor con la gramática proporcionada

    # Ejemplos de cadenas para reconocer
    strings_to_recognize = ["ab", "aab", "aaab", "aaaab"]

    for string in strings_to_recognize:
        if recognizer.recognize(string):
            print(f"'{string}' es reconocida por la gramática de tipo 0.")
        else:
            print(f"'{string}' no es reconocida por la gramática de tipo 0.")
