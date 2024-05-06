#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importa la biblioteca NumPy para trabajar con arreglos y operaciones matemáticas eficientes
import matplotlib.pyplot as plt  # Importa matplotlib para visualización de datos
from sklearn.cluster import KMeans  # Importa la clase KMeans de scikit-learn para el algoritmo de agrupamiento K-Means

# Generar datos aleatorios
np.random.seed(0)  # Establece la semilla aleatoria para reproducibilidad
X = np.random.randn(100, 2)  # Genera 100 muestras con 2 características distribuidas aleatoriamente según una distribución normal

# Crear y ajustar el modelo de K-Means
kmeans = KMeans(n_clusters=3, random_state=0)  # Crea un modelo de K-Means con 3 clusters y una semilla aleatoria
kmeans.fit(X)  # Ajusta el modelo K-Means a los datos X

# Obtener las etiquetas de los clusters y los centroides
labels = kmeans.labels_  # Obtiene las etiquetas de los clusters asignadas a cada muestra
centroids = kmeans.cluster_centers_  # Obtiene las coordenadas de los centroides de los clusters

# Visualizar los datos y los clusters
plt.figure(figsize=(8, 6))  # Establece el tamaño de la figura
plt.scatter(X[:, 0], X[:, 1], c=labels, cmap='viridis', s=50, alpha=0.5)  # Scatter plot de los datos, con color según las etiquetas de los clusters
plt.scatter(centroids[:, 0], centroids[:, 1], marker='x', c='red', s=200, label='Centroides')  # Scatter plot de los centroides
plt.title('Agrupamiento K-Means')  # Establece el título del gráfico
plt.xlabel('Característica 1')  # Etiqueta del eje x
plt.ylabel('Característica 2')  # Etiqueta del eje y
plt.legend()  # Muestra la leyenda
plt.show()  # Muestra la visualización
