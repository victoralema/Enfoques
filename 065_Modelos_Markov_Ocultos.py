#Victor Eduardo Aleman Padilla 21310193

from hmmlearn import hmm
import numpy as np

# Definir el modelo HMM con 2 componentes y matriz de transición simple
modelo = hmm.GaussianHMM(n_components=2, covariance_type="full")

# Datos de entrenamiento (secuencia de observaciones)
X = np.array([[0.5], [1.0], [-1.0], [0.0], [0.5]])

# Ajustar el modelo HMM a los datos de entrenamiento
modelo.fit(X)

# Predecir la secuencia de estados ocultos
estados_ocultos = modelo.predict(X)

# Imprimir los estados ocultos predichos
print("Estados ocultos predichos:", estados_ocultos)
