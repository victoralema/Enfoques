#Victor Eduardo Aleman Padilla 21310193


import numpy as np  # Importa la biblioteca NumPy para trabajar con arreglos y operaciones matemáticas eficientes
import matplotlib.pyplot as plt  # Importa matplotlib para visualización de datos
from sklearn import datasets  # Importa datasets de scikit-learn para generar datos de ejemplo
from sklearn.svm import SVC  # Importa SVC de scikit-learn para el modelo de Máquinas de Vectores de Soporte (SVM)

# Generar datos de ejemplo (círculos concéntricos)
X, y = datasets.make_circles(n_samples=100, noise=0.1, factor=0.2, random_state=42)
# Crea datos de ejemplo con 100 muestras, dos círculos concéntricos, ruido del 10%, y un factor de escala de 0.2, utilizando una semilla aleatoria para reproducibilidad

# Visualizar los datos de ejemplo
plt.figure(figsize=(8, 6))
plt.scatter(X[:, 0], X[:, 1], c=y, cmap='viridis', s=50)  # Scatter plot de las características, con color según las etiquetas y tamaño de punto 50
plt.title('Datos de ejemplo (círculos concéntricos)')  # Establece el título del gráfico
plt.xlabel('Característica 1')  # Etiqueta del eje x
plt.ylabel('Característica 2')  # Etiqueta del eje y
plt.show()  # Muestra la visualización de los datos

# Crear el modelo SVM con un núcleo polinomial de grado 3
svm_modelo = SVC(kernel='poly', degree=3, gamma='auto', random_state=42)
# Crea un modelo SVM con un núcleo polinomial de grado 3, utilizando un valor automático para el parámetro gamma y una semilla aleatoria para reproducibilidad

# Ajustar el modelo SVM a los datos
svm_modelo.fit(X, y)  # Ajusta el modelo SVM a los datos de características X y etiquetas y

# Visualizar los límites de decisión del modelo SVM
def plot_decision_boundary(modelo, X, y):
    h = .02  # Tamaño del paso en la malla
    x_min, x_max = X[:, 0].min() - 0.1, X[:, 0].max() + 0.1  # Límites de las características 1
    y_min, y_max = X[:, 1].min() - 0.1, X[:, 1].max() + 0.1  # Límites de las características 2
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))  # Malla de puntos de la región de interés
    Z = modelo.predict(np.c_[xx.ravel(), yy.ravel()])  # Predice las etiquetas para cada punto en la malla
    Z = Z.reshape(xx.shape)  # Reformatea las etiquetas en la forma de la malla
    plt.contourf(xx, yy, Z, cmap='viridis', alpha=0.3)  # Visualiza los límites de decisión como regiones de fondo coloreadas
    plt.scatter(X[:, 0], X[:, 1], c=y, cmap='viridis', s=50)  # Scatter plot de las características, con color según las etiquetas y tamaño de punto 50

plt.figure(figsize=(8, 6))
plot_decision_boundary(svm_modelo, X, y)  # Visualiza los límites de decisión del modelo SVM
plt.title('Límites de decisión del SVM con núcleo polinomial')  # Establece el título del gráfico
plt.xlabel('Característica 1')  # Etiqueta del eje x
plt.ylabel('Característica 2')  # Etiqueta del eje y
plt.show()  # Muestra la visualización de los límites de decisión
