#Victor Eduardo Aleman Padilla 21310193
import numpy as np
import pandas as pd

# Definir la función para calcular la entropía
def entropy(column):
    _, counts = np.unique(column, return_counts=True)  # Calcula los valores únicos y sus recuentos
    probabilities = counts / counts.sum()  # Calcula las probabilidades de cada valor
    entropy = sum(-p * np.log2(p) for p in probabilities)  # Calcula la entropía
    return entropy

# Definir la función para calcular la ganancia de información
def information_gain(data, feature_name, target_name):
    total_entropy = entropy(data[target_name])  # Calcula la entropía total
    values, counts = np.unique(data[feature_name], return_counts=True)  # Calcula los valores únicos y sus recuentos
    weighted_entropy = sum((counts[i] / sum(counts)) * entropy(data.where(data[feature_name] == val).dropna()[target_name]) for i, val in enumerate(values))  # Calcula la entropía ponderada
    information_gain = total_entropy - weighted_entropy  # Calcula la ganancia de información
    return information_gain

# Definir la función para obtener el mejor atributo para dividir
def best_attribute(data, target_name, attributes):
    gains = [information_gain(data, attr, target_name) for attr in attributes]  # Calcula la ganancia de información para cada atributo
    best_index = np.argmax(gains)  # Encuentra el índice del atributo con la mayor ganancia de información
    return attributes[best_index]  # Devuelve el mejor atributo

# Definir la clase del nodo del árbol de decisión
class Node:
    def __init__(self, attribute):
        self.attribute = attribute
        self.children = {}

# Definir la función para construir el árbol de decisión
def build_tree(data, target_name, attributes):
    if len(np.unique(data[target_name])) == 1:  # Si todas las instancias tienen la misma clasificación, devolver esa clasificación
        return np.unique(data[target_name])[0]

    if len(attributes) == 0:  # Si no quedan atributos para dividir, devolver la clasificación más común
        return np.unique(data[target_name])[np.argmax(np.unique(data[target_name], return_counts=True)[1])]

    best_attr = best_attribute(data, target_name, attributes)  # Obtener el mejor atributo para dividir
    root = Node(best_attr)  # Crear un nodo para el árbol con el mejor atributo

    for value in np.unique(data[best_attr]):  # Para cada valor único del mejor atributo
        sub_data = data.where(data[best_attr] == value).dropna()  # Filtrar las instancias que tienen ese valor
        subtree = build_tree(sub_data, target_name, [attr for attr in attributes if attr != best_attr])  # Construir el subárbol recursivamente
        root.children[value] = subtree  # Agregar el subárbol como hijo del nodo actual

    return root  # Devolver el nodo raíz del árbol

# Función para predecir una instancia de datos utilizando el árbol de decisión construido
def predict(instance, tree):
    while isinstance(tree, Node):  # Mientras el nodo actual sea un nodo del árbol
        attr = tree.attribute  # Obtener el atributo del nodo actual
        value = instance[attr]  # Obtener el valor del atributo para la instancia
        if value in tree.children:  # Si el valor está presente en los hijos del nodo actual
            tree = tree.children[value]  # Moverse al hijo correspondiente
        else:
            return "Clasificación desconocida"  # Si el valor no está presente, devolver clasificación desconocida
    return tree  # Devolver la clasificación obtenida

# Ejemplo de uso
data = pd.DataFrame({
    'Outlook': ['Sunny', 'Sunny', 'Overcast', 'Rainy', 'Rainy', 'Rainy', 'Overcast', 'Sunny', 'Sunny', 'Rainy', 'Sunny', 'Overcast', 'Overcast', 'Rainy'],
    'Temperature': ['Hot', 'Hot', 'Hot', 'Mild', 'Cool', 'Cool', 'Cool', 'Mild', 'Cool', 'Mild', 'Mild', 'Mild', 'Hot', 'Mild'],
    'Humidity': ['High', 'High', 'High', 'High', 'Normal', 'Normal', 'Normal', 'High', 'Normal', 'Normal', 'Normal', 'High', 'Normal', 'High'],
    'Windy': [False, True, False, False, False, True, True, False, False, False, True, True, False, True],
    'Play': ['No', 'No', 'Yes', 'Yes', 'Yes', 'No', 'Yes', 'No', 'Yes', 'Yes', 'Yes', 'Yes', 'Yes', 'No']
})

target_name = 'Play'
attributes = ['Outlook', 'Temperature', 'Humidity', 'Windy']

tree = build_tree(data, target_name, attributes)

instance = {'Outlook': 'Sunny', 'Temperature': 'Cool', 'Humidity': 'Normal', 'Windy': True}
print("Predicción:", predict(instance, tree))
