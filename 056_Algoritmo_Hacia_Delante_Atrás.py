#Victor Eduardo Aleman Padilla 21310193

class RedBayesiana:
    def __init__(self):
        # Inicialización de las probabilidades iniciales de los estados X1 y X2
        self.prob_X1 = {'x1_0': 0.7, 'x1_1': 0.3}  # Probabilidad inicial de X1
        self.prob_X2 = {'x2_0': 0.6, 'x2_1': 0.4}  # Probabilidad inicial de X2
        
        # Inicialización de las probabilidades de la evidencia dado los estados X1 y X2
        self.prob_E_given_X1_X2 = {
            ('x1_0', 'x2_0'): 0.8,  # Probabilidad de E dado X1=0 y X2=0
            ('x1_0', 'x2_1'): 0.3,  # Probabilidad de E dado X1=0 y X2=1
            ('x1_1', 'x2_0'): 0.2,  # Probabilidad de E dado X1=1 y X2=0
            ('x1_1', 'x2_1'): 0.7   # Probabilidad de E dado X1=1 y X2=1
        }

    def forward_backward(self, evidence):
        # Paso hacia adelante (forward)
        alpha = {}
        for x1 in ['x1_0', 'x1_1']:
            for x2 in ['x2_0', 'x2_1']:
                # Calcula la probabilidad conjunta de la evidencia y los estados X1 y X2
                alpha[(x1, x2)] = self.prob_X1[x1] * self.prob_X2[x2] * self.prob_E_given_X1_X2[(x1, x2)]

        # Paso hacia atrás (backward)
        beta = {}
        for x1 in ['x1_0', 'x1_1']:
            for x2 in ['x2_0', 'x2_1']:
                # Inicializa las contribuciones hacia atrás como 1.0
                beta[(x1, x2)] = 1.0

        # Normalización de las probabilidades calculadas en el paso hacia adelante
        evidence_sum = sum(alpha.values())
        for key in alpha:
            alpha[key] /= evidence_sum

        # Actualización de la creencia utilizando las probabilidades calculadas en ambos pasos
        belief = {}
        for x1 in ['x1_0', 'x1_1']:
            for x2 in ['x2_0', 'x2_1']:
                # Combinación de las probabilidades hacia adelante y hacia atrás
                belief[(x1, x2)] = alpha[(x1, x2)] * beta[(x1, x2)]

        # Normalización de la creencia posterior
        belief_sum = sum(belief.values())
        for key in belief:
            belief[key] /= belief_sum

        return belief


# Ejemplo de uso
red_bayesiana = RedBayesiana()
evidencia = {'e_0': 1.0, 'e_1': 0.0}  # Supongamos que observamos E=0
creencia = red_bayesiana.forward_backward(evidencia)
print("Creencia posterior:")
for key, value in creencia.items():
    print(f"{key}: {value}")
