#Victor Eduardo Aleman Padilla 21310193
import itertools  # Importa la librería itertools para generar combinaciones

def generar_tabla_verdad(expresion):
    # Extraer las variables únicas de la expresión
    variables = sorted(set([char for char in expresion if char.isalpha()]))  # Encuentra todas las letras en la expresión y las convierte en un conjunto único

    # Imprimir encabezados de las columnas
    encabezados = variables + [expresion]  # Define los encabezados de la tabla de verdad
    print("\t".join(encabezados))  # Imprime los encabezados de las columnas separados por tabulaciones

    # Generar todas las combinaciones posibles de valores de verdad
    combinaciones = list(itertools.product([False, True], repeat=len(variables)))  # Genera todas las combinaciones de True y False para las variables

    # Calcular el valor de verdad para cada combinación
    for combo in combinaciones:  # Para cada combinación de valores de verdad
        valores = dict(zip(variables, combo))  # Crea un diccionario que mapea cada variable a su valor de verdad correspondiente en esta combinación
        resultado = eval(expresion, valores)  # Evalúa la expresión lógica con los valores de verdad dados
        valores_lista = [str(valores[var]) for var in variables] + [str(resultado)]  # Crea una lista con los valores de verdad de las variables y el resultado de la expresión
        print("\t".join(valores_lista))  # Imprime los valores de verdad de las variables y el resultado, separados por tabulaciones

# Ejemplo de uso
expresion_logica = "(A and B) or (not A)"
generar_tabla_verdad(expresion_logica)
