#Victor Eduardo Aleman Padilla 21310193
import sympy as sp  # Importar la biblioteca sympy para trabajar con lógica simbólica

def resolucion(formula1, formula2):
    # Convertir las fórmulas a expresiones simbólicas
    expr1 = sp.parse_expr(formula1)  # Convertir la primera fórmula a expresión simbólica
    expr2 = sp.parse_expr(formula2)  # Convertir la segunda fórmula a expresión simbólica
    # Obtener la forma normal conjuntiva de ambas expresiones
    fnc_expr1 = sp.to_cnf(expr1)  # Convertir la primera expresión a Forma Normal Conjuntiva (FNC)
    fnc_expr2 = sp.to_cnf(expr2)  # Convertir la segunda expresión a Forma Normal Conjuntiva (FNC)
    # Resolver usando la regla de resolución
    resol = sp.satisfiable(sp.And(fnc_expr1, fnc_expr2))  # Resolver utilizando la regla de resolución
    return resol

def fnc(formula):
    # Convertir la fórmula a una expresión simbólica
    expr = sp.parse_expr(formula)  # Convertir la fórmula a una expresión simbólica
    # Convertir a FNC
    fnc_expr = sp.to_cnf(expr)  # Convertir la expresión a Forma Normal Conjuntiva (FNC)
    return fnc_expr

if __name__ == "__main__":
    print("Bienvenido al programa de lógica proposicional")

    # Ejemplo de resolución
    formula1 = input("Introduce la primera fórmula: ")  # Solicitar al usuario que introduzca la primera fórmula
    formula2 = input("Introduce la segunda fórmula: ")  # Solicitar al usuario que introduzca la segunda fórmula
    resultado_resolucion = resolucion(formula1, formula2)  # Llamar a la función de resolución
    if resultado_resolucion:
        print("Las fórmulas son resolubles.")  # Imprimir si las fórmulas son resolubles
    else:
        print("Las fórmulas no son resolubles.")  # Imprimir si las fórmulas no son resolubles

    # Ejemplo de conversión a FNC
    formula_fnc = input("Introduce la fórmula para convertir a FNC: ")  # Solicitar al usuario que introduzca la fórmula
    fnc_resultado = fnc(formula_fnc)  # Llamar a la función de conversión a FNC
    print("La FNC de la fórmula es:", fnc_resultado)  # Imprimir la FNC de la fórmula

"""
Introduce la primera fórmula: p | q
Introduce la segunda fórmula: ~p | r
Las fórmulas son resolubles.
Introduce la fórmula para convertir a FNC: (p & q) | (~r)
La FNC de la fórmula es: Or(And(p, q), Not(r))
"""
