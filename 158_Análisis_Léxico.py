#Victor Eduardo Aleman Padilla 21310193
import re  # Importamos el módulo de expresiones regulares de Python

class Lexer:
    def __init__(self, rules):
        self.rules = rules  # Inicializamos el Lexer con un conjunto de reglas para reconocer tokens

    def tokenize(self, input_string):
        tokens = []  # Creamos una lista para almacenar los tokens encontrados
        while input_string:  # Mientras haya entrada por procesar
            match = None  # Inicializamos la variable de coincidencia de patrón
            for token_type, pattern in self.rules.items():  # Iteramos sobre las reglas de token
                regex = re.compile(pattern)  # Compilamos la expresión regular para el patrón actual
                match = regex.match(input_string)  # Intentamos hacer coincidir el patrón con la entrada
                if match:  # Si hay una coincidencia
                    value = match.group(0)  # Obtenemos el valor coincidente
                    tokens.append((token_type, value))  # Añadimos el token encontrado a la lista de tokens
                    input_string = input_string[len(value):].lstrip()  # Actualizamos la entrada para omitir el token encontrado
                    break  # Salimos del bucle de reglas
            if not match:  # Si no se encontró ninguna coincidencia de patrón
                raise ValueError("No se pudo reconocer el token en la entrada: " + input_string)  # Lanzamos una excepción
        return tokens  # Devolvemos la lista de tokens encontrados

# Ejemplo de uso
if __name__ == "__main__":
    # Definimos las reglas de token como un diccionario donde las claves son los tipos de token y los valores son las expresiones regulares
    rules = {
        'NUMBER': r'\d+',  # Coincide con uno o más dígitos
        'PLUS': r'\+',  # Coincide con el símbolo de suma
        'MINUS': r'-',  # Coincide con el símbolo de resta
        'TIMES': r'\*',  # Coincide con el símbolo de multiplicación
        'DIVIDE': r'/',  # Coincide con el símbolo de división
        'LPAREN': r'\(',  # Coincide con el paréntesis izquierdo
        'RPAREN': r'\)',  # Coincide con el paréntesis derecho
    }

    # Creamos un objeto Lexer con las reglas definidas
    lexer = Lexer(rules)

    # Cadena de entrada para tokenizar
    input_string = "3 + 4 * (10 - 6)"

    # Tokenizamos la cadena de entrada
    tokens = lexer.tokenize(input_string)

    # Imprimimos los tokens encontrados
    print("Tokens:")
    for token in tokens:
        print(token)
