#Victor Eduardo Aleman Padilla 21310193
class VersionSpace:
    def __init__(self, attributes):
        # Inicializa el espacio de versiones con una lista de atributos.
        self.attributes = attributes
        # Inicializa la lista de hipótesis con una hipótesis inicial vacía.
        self.hypotheses = [{}]

    def specialize(self, example):
        # Crea nuevas hipótesis especializadas a partir de un ejemplo positivo.
        new_hypotheses = []
        for hypothesis in self.hypotheses:
            for attr, value in example.items():
                if attr in hypothesis:
                    if hypothesis[attr] != value:
                        new_hypothesis = hypothesis.copy()
                        new_hypothesis.pop(attr)
                        new_hypotheses.append(new_hypothesis)
                else:
                    new_hypothesis = hypothesis.copy()
                    new_hypothesis[attr] = value
                    new_hypotheses.append(new_hypothesis)
        # Actualiza la lista de hipótesis con las nuevas hipótesis especializadas.
        self.hypotheses = new_hypotheses

    def generalize(self, example):
        # Generaliza las hipótesis basándose en un ejemplo negativo.
        self.hypotheses = [hypothesis for hypothesis in self.hypotheses if all(example.get(attr) == value or value is None for attr, value in hypothesis.items())]

    def predict(self, example):
        # Predice si un ejemplo dado pertenece al espacio de versiones.
        for hypothesis in self.hypotheses:
            if all(example.get(attr) == value for attr, value in hypothesis.items()):
                return True
        return False

class AQ:
    def __init__(self, examples, target_attribute):
        # Inicializa el algoritmo AQ con ejemplos de entrenamiento y el atributo objetivo.
        self.examples = examples
        self.target_attribute = target_attribute

    def train(self):
        # Entrena el espacio de versiones con los ejemplos de entrenamiento.
        vs = VersionSpace(self.examples[0].keys())
        for example in self.examples:
            if example[self.target_attribute]:
                vs.specialize(example)
            else:
                vs.generalize(example)
        return vs

# Ejemplo de uso:
examples = [
    {'Sunny': True, 'Warm': True, 'High': False, 'Strong': False, 'PlayTennis': True},
    {'Sunny': True, 'Warm': True, 'High': False, 'Strong': True, 'PlayTennis': True},
    {'Overcast': True, 'Warm': True, 'High': False, 'Strong': False, 'PlayTennis': True},
    {'Rainy': True, 'Cool': False, 'High': False, 'Strong': False, 'PlayTennis': True},
    {'Rainy': False, 'Cool': False, 'High': True, 'Strong': False, 'PlayTennis': False},
]

# Inicializa el algoritmo AQ con ejemplos y el atributo objetivo.
aq = AQ(examples, 'PlayTennis')
# Entrena el algoritmo y obtiene el espacio de versiones resultante.
vs = aq.train()

# Imprime las hipótesis en el espacio de versiones.
print("Hypotheses in the version space:")
for hypothesis in vs.hypotheses:
    print(hypothesis)

# Predicción de una nueva instancia.
new_instance = {'Sunny': True, 'Warm': True, 'High': False, 'Strong': False}
prediction = vs.predict(new_instance)
print("Prediction for new instance:", prediction)
