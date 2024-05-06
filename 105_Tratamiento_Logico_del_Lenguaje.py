#Victor Eduardo Aleman Padilla 21310193
import nltk  # Importa la biblioteca NLTK (Natural Language Toolkit)
from nltk.tokenize import word_tokenize  # Importa el tokenizador de palabras de NLTK
from nltk.corpus import stopwords  # Importa la lista de stopwords de NLTK

# Descargar los recursos necesarios de NLTK (solo la primera vez que se ejecuta el programa)
nltk.download('punkt')  # Descargar el tokenizador de palabras
nltk.download('stopwords')  # Descargar la lista de stopwords

# Definir una función para clasificar un texto basado en palabras clave
def classify_text(text):
    # Tokenizar el texto en palabras
    words = word_tokenize(text)  # Utilizar el tokenizador de palabras para dividir el texto en palabras individuales
    
    # Obtener las palabras clave eliminando las stopwords (palabras comunes que no aportan mucho significado)
    keywords = set(words) - set(stopwords.words('english'))  # Crear un conjunto de palabras únicas excluyendo las stopwords en inglés
    
    # Clasificar el texto basado en las palabras clave
    if 'sports' in keywords:  # Si la palabra 'sports' está en las palabras clave
        return 'Sports'  # Clasificar como deportes
    elif 'technology' in keywords:  # Si la palabra 'technology' está en las palabras clave
        return 'Technology'  # Clasificar como tecnología
    else:  # Para cualquier otro caso
        return 'Other'  # Clasificar como otro tipo de texto

# Ejemplo de uso
text1 = "I love playing basketball."  # Texto relacionado con deportes
text2 = "The new iPhone has some amazing features."  # Texto relacionado con tecnología
text3 = "The weather is beautiful today."  # Texto no relacionado

print("Clasificación de textos:")
print("Texto 1:", classify_text(text1))  # Clasificar el texto 1 y mostrar la clasificación
print("Texto 2:", classify_text(text2))  # Clasificar el texto 2 y mostrar la clasificación
print("Texto 3:", classify_text(text3))  # Clasificar el texto 3 y mostrar la clasificación
