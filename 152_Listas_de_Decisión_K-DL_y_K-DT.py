#Victor Eduardo Aleman Padilla 21310193
# Importar las bibliotecas necesarias de scikit-learn
from sklearn.datasets import load_iris  # Para cargar el conjunto de datos Iris
from sklearn.model_selection import train_test_split  # Para dividir el conjunto de datos en entrenamiento y prueba
from sklearn.metrics import accuracy_score  # Para calcular la precisión del modelo
from sklearn.neighbors import KNeighborsClassifier  # Para el algoritmo K-DL (Lista de Decisión K)
from sklearn.tree import DecisionTreeClassifier  # Para el algoritmo K-DT (Árbol de Decisión K)

# Cargar el conjunto de datos Iris
iris = load_iris()
X = iris.data  # Características de las flores
y = iris.target  # Etiquetas de las flores

# Dividir el conjunto de datos en entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)

# Algoritmo K-DL (Lista de Decisión K)
k_dl_model = KNeighborsClassifier(n_neighbors=3)  # Definir el modelo K-DL con K=3
k_dl_model.fit(X_train, y_train)  # Entrenar el modelo K-DL
k_dl_pred = k_dl_model.predict(X_test)  # Hacer predicciones con el modelo K-DL
k_dl_accuracy = accuracy_score(y_test, k_dl_pred)  # Calcular la precisión del modelo K-DL
print("K-DL Accuracy:", k_dl_accuracy)  # Imprimir la precisión del modelo K-DL

# Algoritmo K-DT (Árbol de Decisión K)
k_dt_model = DecisionTreeClassifier(max_depth=3)  # Definir el modelo K-DT con una profundidad máxima de 3
k_dt_model.fit(X_train, y_train)  # Entrenar el modelo K-DT
k_dt_pred = k_dt_model.predict(X_test)  # Hacer predicciones con el modelo K-DT
k_dt_accuracy = accuracy_score(y_test, k_dt_pred)  # Calcular la precisión del modelo K-DT
print("K-DT Accuracy:", k_dt_accuracy)  # Imprimir la precisión del modelo K-DT
