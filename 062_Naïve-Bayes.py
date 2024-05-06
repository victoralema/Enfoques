#Victor Eduardo Aleman Padilla 21310193

from sklearn.feature_extraction.text import CountVectorizer  # Importa la clase CountVectorizer de scikit-learn para convertir el texto en una matriz de frecuencia de términos
from sklearn.naive_bayes import MultinomialNB  # Importa el clasificador Naïve Bayes multinomial de scikit-learn
from sklearn.pipeline import make_pipeline  # Importa la función make_pipeline de scikit-learn para construir un pipeline de procesamiento de datos

# Datos de entrenamiento
textos_entrenamiento = ["comprar pastillas para adelgazar", "oferta exclusiva, ¡gana un millón de dólares!",
                        "reunión mañana por la mañana", "última llamada para actualizar su información de cuenta"]
etiquetas_entrenamiento = ["spam", "spam", "no_spam", "no_spam"]

# Crear el modelo Naïve Bayes
modelo = make_pipeline(CountVectorizer(), MultinomialNB())  # Crea un pipeline que incluye un CountVectorizer y un clasificador Naïve Bayes multinomial

# Entrenar el modelo
modelo.fit(textos_entrenamiento, etiquetas_entrenamiento)  # Entrena el modelo con los datos de entrenamiento

# Documento de prueba
documento_prueba = ["comprar pastillas mañana por la mañana"]

# Predecir la clase del documento de prueba
clase_predicha = modelo.predict(documento_prueba)  # Utiliza el modelo entrenado para predecir la clase del documento de prueba

print("El documento se clasifica como:", clase_predicha[0])  # Imprime la clase predicha del documento de prueba
