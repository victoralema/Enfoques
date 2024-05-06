#Victor Eduardo Aleman Padilla 21310193

import numpy as np

# Definir las probabilidades iniciales
# Supongamos que tenemos dos posibles estados: A y B
# Definimos las probabilidades iniciales P(A) y P(B)
prob_A = 0.5
prob_B = 0.5

# Definir la matriz de probabilidades de transición
# Probabilidades de transición de estado a estado
# Por ejemplo, P(A|A), P(A|B), P(B|A), P(B|B)
transition_matrix = np.array([[0.7, 0.3],
                               [0.4, 0.6]])

def bayesian_inference(evidence, prior_prob):
    """
    Realiza inferencia bayesiana dada una evidencia y probabilidades a priori.
    
    Args:
    - evidence (str): La evidencia observada ('A' o 'B').
    - prior_prob (float): La probabilidad a priori de la hipótesis.
    
    Returns:
    - posterior_prob (float): La probabilidad a posteriori de la hipótesis dado la evidencia.
    """
    global prob_A, prob_B, transition_matrix
    
    if evidence == 'A':
        likelihood_A = transition_matrix[0, 0]
        likelihood_B = transition_matrix[1, 0]
    elif evidence == 'B':
        likelihood_A = transition_matrix[0, 1]
        likelihood_B = transition_matrix[1, 1]
    
    # Aplicar regla de Bayes
    evidence_prob = likelihood_A * prob_A + likelihood_B * prob_B
    posterior_prob = (likelihood_A * prob_A) / evidence_prob
    
    # Actualizar las probabilidades a priori para la siguiente iteración
    prob_A = posterior_prob
    prob_B = 1 - prob_A
    
    return posterior_prob

# Simular una serie de observaciones
observations = ['A', 'A', 'B', 'A', 'B']

# Realizar inferencia bayesiana para cada observación
for obs in observations:
    posterior = bayesian_inference(obs, prob_A)
    print(f"Después de observar '{obs}', la probabilidad de estar en estado A es: {posterior:.4f}")

# Imprimir la probabilidad final de estar en el estado A
print(f"\nProbabilidad final de estar en estado A: {prob_A:.4f}")
