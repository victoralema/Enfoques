#Victor Eduardo Aleman Padilla 21310193
# Importar la biblioteca sympy para trabajar con lógica simbólica
import sympy as sp

# Función para verificar la equivalencia entre dos fórmulas lógicas
def equivalencia(formula1, formula2):
    # Convertir las fórmulas a expresiones simbólicas
    expr1 = sp.parse_expr(formula1)
    expr2 = sp.parse_expr(formula2)
    # Verificar si las expresiones son lógicamente equivalentes
    return sp.Equivalent(expr1, expr2)

# Función para verificar la validez de una fórmula lógica
def validez(formula):
    # Convertir la fórmula a una expresión simbólica
    expr = sp.parse_expr(formula)
    # Verificar si la expresión es válida en todas las interpretaciones
    return sp.satisfiable(expr)

# Función para verificar la satisfacibilidad de una fórmula lógica
def satisfacibilidad(formula):
    # Convertir la fórmula a una expresión simbólica
    expr = sp.parse_expr(formula)
    # Verificar si la expresión es satisfacible, es decir, si existe alguna interpretación que la haga verdadera
    return sp.satisfiable(expr)

# Bloque principal del programa
if __name__ == "__main__":
    print("Bienvenido al programa de lógica proposicional")

    # Ejemplo de equivalencia
    formula1 = input("Introduce la primera fórmula: ")
    formula2 = input("Introduce la segunda fórmula: ")
    if equivalencia(formula1, formula2):
        print("Las fórmulas son equivalentes.")
    else:
        print("Las fórmulas no son equivalentes.")

    # Ejemplo de validez
    formula_validez = input("Introduce la fórmula para verificar su validez: ")
    if validez(formula_validez):
        print("La fórmula es válida.")
    else:
        print("La fórmula no es válida.")

    # Ejemplo de satisfacibilidad
    formula_satisfacibilidad = input("Introduce la fórmula para verificar su satisfacibilidad: ")
    if satisfacibilidad(formula_satisfacibilidad):
        print("La fórmula es satisfacible.")
    else:
        print("La fórmula no es satisfacible.")
