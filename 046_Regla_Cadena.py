#Victor Eduardo Aleman Padilla 21310193

def calcular_probabilidad_condicional(p_a, p_b_dado_a):
    # Calcular la probabilidad conjunta P(A, B) usando la Regla de la Cadena
    p_a_y_b = p_a * p_b_dado_a
    return p_a_y_b

def main():
    # Pedir al usuario que ingrese las probabilidades P(A) y P(B|A)
    p_a = float(input("Ingrese la probabilidad de A (P(A)): "))
    p_b_dado_a = float(input("Ingrese la probabilidad de B dado A (P(B|A)): "))

    # Calcular la probabilidad conjunta P(A, B)
    p_a_y_b = calcular_probabilidad_condicional(p_a, p_b_dado_a)

    # Mostrar el resultado
    print("La probabilidad conjunta P(A, B) es:", p_a_y_b)

if __name__ == "__main__":
    main()
