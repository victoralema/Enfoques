#Victor Eduardo Aleman Padilla 21310193
# Importamos las funciones y clases necesarias de la biblioteca sympy
from sympy import symbols, Or, And, Not, simplify
from sympy.logic.boolalg import to_cnf

# Definimos los símbolos proposicionales
A, B, C, D = symbols('A B C D')

# Definimos las cláusulas
clausulas = [Or(A, B, C),    # Primera cláusula
             Or(Not(A), D),   # Segunda cláusula
             Or(Not(B), D),   # Tercera cláusula
             Or(Not(C), D)]   # Cuarta cláusula

# Convertimos las cláusulas a la Forma Normal Conjuntiva (CNF)
cnf = to_cnf(And(*clausulas))

# Definimos una función para realizar la resolución
def resolucion(cnf):
    while True:  # Iteramos indefinidamente hasta que se alcance una condición de salida
        new_clauses = set()  # Creamos un conjunto para almacenar las nuevas cláusulas generadas
        # Seleccionamos dos cláusulas de la CNF
        for i, clause1 in enumerate(cnf.args):
            for j, clause2 in enumerate(cnf.args):
                if i != j:  # Aseguramos que las cláusulas sean diferentes
                    # Buscamos un literal que sea complementario en ambas cláusulas
                    for literal in clause1.args:
                        if Not(literal) in clause2.args:
                            # Agregamos la resolvente (la cláusula resultante)
                            resolvente = simplify(Or(*(clause1.args + clause2.args)).simplify())
                            new_clauses.add(resolvente)
                            break
        # Verificamos si encontramos una cláusula vacía (insatisfacibilidad)
        if set() in new_clauses:
            return "Insatisfacible"
        # Comprobamos si las nuevas cláusulas ya están en la CNF
        if new_clauses.issubset(cnf.args):
            return "Satisfacible"
        # Actualizamos las cláusulas en CNF
        cnf |= And(*new_clauses)

# Ejecutamos la función de resolución y almacenamos el resultado
resultado = resolucion(cnf)
# Imprimimos el resultado
print("El resultado es:", resultado)
