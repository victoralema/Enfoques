#Victor Eduardo Aleman Padilla 21310193

from sklearn.datasets import load_iris  # Importar la función para cargar el conjunto de datos Iris
from sklearn.model_selection import train_test_split  # Importar la función para dividir los datos en conjuntos de entrenamiento y prueba

# Cargar el conjunto de datos Iris
iris = load_iris()  # Cargar los datos del conjunto de datos Iris
X = iris.data  # Extraer las características de las flores
y = iris.target  # Extraer las etiquetas de las clases de las flores

# Dividir los datos en conjuntos de entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)  # Dividir los datos en 80% de entrenamiento y 20% de prueba

# Definir una función para clasificar una flor basada en reglas
def classify_flower(features):
    # Extraer las características de la flor
    sepal_length, sepal_width, petal_length, petal_width = features
    
    # Reglas de clasificación basadas en los datos del conjunto de entrenamiento
    if petal_width < 0.75:  # Si el ancho del pétalo es menor que 0.75
        return 'Setosa'  # Clasificar como Setosa
    elif petal_length < 4.75:  # Si la longitud del pétalo es menor que 4.75
        return 'Versicolor'  # Clasificar como Versicolor
    else:  # Para todas las demás flores
        return 'Virginica'  # Clasificar como Virginica

# Clasificar las flores en el conjunto de prueba utilizando la función de clasificación definida
predictions = [classify_flower(features) for features in X_test]

# Calcular la precisión del clasificador comparando las predicciones con las etiquetas reales
accuracy = sum(1 for pred, true in zip(predictions, y_test) if pred == iris.target_names[true]) / len(y_test)

print("Precisión del clasificador:", accuracy)  # Imprimir la precisión del clasificador
