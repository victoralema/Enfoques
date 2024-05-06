#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importa la biblioteca NumPy para trabajar con arreglos y operaciones matemáticas eficientes
from sklearn.mixture import GaussianMixture  # Importa la clase GaussianMixture de scikit-learn para modelar una mezcla de Gaussianas

# Datos de ejemplo (dos componentes gaussianas)
np.random.seed(0)  # Establece la semilla aleatoria para reproducibilidad
X = np.concatenate([np.random.normal(0, 1, 1000), np.random.normal(4, 1, 1000)]).reshape(-1, 1)
# Genera 1000 muestras de dos distribuciones normales (una con media 0 y desviación estándar 1, y otra con media 4 y desviación estándar 1)
# Luego, concatena las muestras y las redimensiona en un arreglo bidimensional

# Crear y ajustar el modelo de mezcla de Gaussianas
modelo = GaussianMixture(n_components=2, random_state=0)  # Crea un modelo de mezcla de Gaussianas con dos componentes y una semilla aleatoria
modelo.fit(X)  # Ajusta el modelo a los datos X

# Imprimir los parámetros estimados de las distribuciones gaussianas
print("Parámetros estimados de la distribución 1:")
print("Media:", modelo.means_[0])  # Imprime la media estimada de la primera distribución gaussiana
print("Desviación estándar:", np.sqrt(modelo.covariances_[0]))  # Imprime la desviación estándar estimada de la primera distribución gaussiana
print()
print("Parámetros estimados de la distribución 2:")
print("Media:", modelo.means_[1])  # Imprime la media estimada de la segunda distribución gaussiana
print("Desviación estándar:", np.sqrt(modelo.covariances_[1]))  # Imprime la desviación estándar estimada de la segunda distribución gaussiana
