#Victor Eduardo Aleman Padilla 21310193
from sympy import symbols, Implies  # Importamos los símbolos y el operador de implicación de SymPy

# Creamos símbolos para representar las variables en la lógica de primer orden
x, y = symbols('x y')

# Definimos algunos axiomas utilizando implicaciones
axioms = [
    Implies(x, y),                    # x implica y
    Implies(y, x),                    # y implica x
    Implies(x, y) & Implies(y, x),    # x implica y y y implica x (equivalente a x si y)
    Implies(x, y) | Implies(y, x)    # x implica y o y implica x
]

def main():
    print("Axioms:")
    for axiom in axioms:
        print(axiom)  # Imprimimos cada axioma

    # Ejemplo de inferencia: x implica y, y implica z, entonces x implica z
    inference = Implies(x, y) & Implies(y, symbols('z'))
    print("\nInference:")
    print(inference)  # Imprimimos la inferencia

if __name__ == "__main__":
    main()  # Llamamos a la función main si el script es ejecutado directamente
