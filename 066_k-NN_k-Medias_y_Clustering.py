#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importa la biblioteca NumPy para trabajar con arreglos y operaciones matemáticas eficientes
import matplotlib.pyplot as plt  # Importa matplotlib para visualización de datos
from sklearn.datasets import make_blobs  # Importa make_blobs para generar datos de ejemplo
from sklearn.neighbors import KNeighborsClassifier  # Importa KNeighborsClassifier para el algoritmo k-NN
from sklearn.cluster import KMeans  # Importa KMeans para el algoritmo k-Medias

# Generar datos de ejemplo con 4 centroides
X, y = make_blobs(n_samples=300, centers=4, random_state=42)
# Crea datos de ejemplo con 300 muestras y 4 centroides, utilizando una semilla aleatoria para reproducibilidad

# Visualizar los datos de ejemplo
plt.figure(figsize=(10, 5))
plt.scatter(X[:, 0], X[:, 1], c=y, s=50, cmap='viridis')
plt.title('Datos de ejemplo')
plt.xlabel('Característica 1')
plt.ylabel('Característica 2')
plt.show()
# Visualiza los datos de ejemplo en un scatter plot, coloreando cada punto según su etiqueta y mostrando el título y etiquetas de los ejes

# Algoritmo k-NN
knn_modelo = KNeighborsClassifier(n_neighbors=3)  # Crea un modelo k-NN con 3 vecinos
knn_modelo.fit(X, y)  # Ajusta el modelo k-NN a los datos

# Algoritmo k-Medias
kmeans_modelo = KMeans(n_clusters=4)  # Crea un modelo k-Medias con 4 clusters
kmeans_modelo.fit(X)  # Ajusta el modelo k-Medias a los datos

# Visualizar los resultados de k-NN y k-Medias
plt.figure(figsize=(18, 5))

# Visualizar los resultados de k-NN
plt.subplot(1, 3, 1)
plt.scatter(X[:, 0], X[:, 1], c=knn_modelo.predict(X), s=50, cmap='viridis')
plt.title('k-NN')
plt.xlabel('Característica 1')
plt.ylabel('Característica 2')

# Visualizar los resultados de k-Medias
plt.subplot(1, 3, 2)
plt.scatter(X[:, 0], X[:, 1], c=kmeans_modelo.labels_, s=50, cmap='viridis')
plt.scatter(kmeans_modelo.cluster_centers_[:, 0], kmeans_modelo.cluster_centers_[:, 1], c='red', s=200, alpha=0.5, label='Centroides')
plt.title('k-Medias')
plt.xlabel('Característica 1')
plt.ylabel('Característica 2')
plt.legend()

plt.show()
# Visualiza los resultados de k-NN y k-Medias en dos subplots, con los datos coloreados por cluster y los centroides de los clusters resaltados en rojo
