#Victor Eduardo Aleman Padilla 21310193
import numpy as np
from sklearn.datasets import fetch_openml
from sklearn.model_selection import train_test_split
from sklearn.naive_bayes import GaussianNB
from sklearn.metrics import accuracy_score

# Importar bibliotecas necesarias

# Cargar el conjunto de datos MNIST
mnist = fetch_openml('mnist_784', version=1)

# Obtener características (imágenes) y etiquetas del conjunto de datos
X = mnist.data.astype('float32')  # Características (imágenes)
y = mnist.target.astype('int')     # Etiquetas

# Dividir los datos en conjunto de entrenamiento y conjunto de prueba
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Crear un clasificador de Bayes Ingenuo Gaussiano
naive_bayes = GaussianNB()

# Entrenar el clasificador utilizando el conjunto de entrenamiento
naive_bayes.fit(X_train, y_train)

# Realizar predicciones sobre el conjunto de prueba
y_pred = naive_bayes.predict(X_test)

# Calcular la precisión del modelo
accuracy = accuracy_score(y_test, y_pred)

# Imprimir la precisión del modelo
print("Precisión del clasificador de Bayes Ingenuo: {:.2f}%".format(accuracy * 100))
