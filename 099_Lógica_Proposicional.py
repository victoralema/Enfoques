#Victor Eduardo Aleman Padilla 21310193
class LogicEvaluator:
    def __init__(self, variables):
        self.variables = variables  # Inicializa la instancia del evaluador con un diccionario de variables y sus valores

    def evaluate(self, expression):
        # Reemplaza las variables en la expresión con sus valores correspondientes
        for var, value in self.variables.items():
            expression = expression.replace(var, str(value))

        # Evalúa la expresión lógica resultante y devuelve el resultado
        return eval(expression)  # Utiliza la función eval() para evaluar la expresión lógica

def main():
    # Definimos las variables y sus valores (True o False)
    variables = {'p': True, 'q': False}

    # Creamos una instancia del evaluador lógico
    evaluator = LogicEvaluator(variables)

    # Ejemplos de expresiones lógicas para evaluar
    expressions = ['p and q', 'p or q', 'not p', 'p and not q']

    # Evaluamos cada expresión e imprimimos el resultado
    for expr in expressions:
        result = evaluator.evaluate(expr)  # Llama al método evaluate() para evaluar la expresión
        print(f"{expr}: {result}")  # Imprime la expresión y su resultado

if __name__ == "__main__":
    main()  # Llama a la función main si el script es ejecutado directamente
