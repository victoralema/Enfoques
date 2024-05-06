#Victor Eduardo Aleman Padilla 21310193

def prob_condicional(A, B, P_A, P_B_dado_A):
    """
    Calcula la probabilidad condicional P(B|A) y verifica si A y B son independientes.
    
    Args:
    A: bool, evento A (True si ocurre, False si no ocurre)
    B: bool, evento B (True si ocurre, False si no ocurre)
    P_A: float, probabilidad de que ocurra A (P(A))
    P_B_dado_A: float, probabilidad de que ocurra B dado que A ocurre (P(B|A))
    
    Returns:
    float, probabilidad condicional P(B|A)
    bool, True si A y B son independientes, False si no son independientes
    """
    # Calcula P(B|A) usando la fórmula de probabilidad condicional
    P_B_cond_A = P_B_dado_A if A else 0.0  # P(B|A) = P(B) si A es verdadero, 0 si A es falso
    
    # Calcula P(B) usando la regla del producto
    P_B = P_B_cond_A * P_A + (1 - P_A) * 0.0  # P(B) = P(B|A)P(A) + P(B|¬A)P(¬A)
    
    # Verifica si A y B son independientes comparando P(B|A) con P(B)
    es_independiente = abs(P_B_cond_A - P_B) < 1e-9  # Usamos una pequeña tolerancia para la comparación
    
    return P_B_cond_A, es_independiente

# Ejemplo de uso
if __name__ == "__main__":
    # Definir los eventos y sus probabilidades
    A = True  # Evento A ocurre
    B = True  # Evento B ocurre
    P_A = 0.3  # Probabilidad de que ocurra A (P(A))
    P_B_dado_A = 0.4  # Probabilidad de que ocurra B dado que A ocurre (P(B|A))
    
    # Calcular la probabilidad condicional P(B|A) y verificar independencia
    P_B_cond_A, independiente = prob_condicional(A, B, P_A, P_B_dado_A)
    
    # Imprimir resultados
    print(f"La probabilidad condicional P(B|A) es: {P_B_cond_A:.2f}")
    if independiente:
        print("Los eventos A y B son independientes.")
    else:
        print("Los eventos A y B no son independientes.")
