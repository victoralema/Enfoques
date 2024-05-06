#Victor Eduardo Aleman Padilla 21310193

def calcular_probabilidad_bayes(probabilidad_A, probabilidad_B_dado_A, probabilidad_B):
    # Calcula la probabilidad condicional P(A|B) usando la Regla de Bayes
    # P(A|B) = (P(B|A) * P(A)) / P(B)

    # Probabilidad de A
    P_A = probabilidad_A
    
    # Probabilidad de B dado A
    P_B_dado_A = probabilidad_B_dado_A
    
    # Probabilidad de B
    P_B = probabilidad_B
    
    # Aplicar la fórmula de Bayes
    P_A_dado_B = (P_B_dado_A * P_A) / P_B
    
    return P_A_dado_B

# Ejemplo de uso
if __name__ == "__main__":
    # Supongamos que tenemos dos eventos A y B
    # Queremos calcular P(A|B)

    # Probabilidad de A
    P_A = 0.3

    # Probabilidad de B dado A
    P_B_dado_A = 0.6

    # Probabilidad de B
    P_B = 0.5

    # Calcular P(A|B) usando la Regla de Bayes
    resultado = calcular_probabilidad_bayes(P_A, P_B_dado_A, P_B)

    # Imprimir el resultado
    print(f"La probabilidad de A dado B es: {resultado}")
