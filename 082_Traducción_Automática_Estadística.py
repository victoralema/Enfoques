#Victor Eduardo Aleman Padilla 21310193
import random

# Diccionario de traducción
diccionario = {
    "hello": {"fr": "bonjour", "es": "hola", "de": "hallo"},  # Traducciones de "hello" a diferentes idiomas
    "goodbye": {"fr": "au revoir", "es": "adiós", "de": "auf Wiedersehen"},  # Traducciones de "goodbye" a diferentes idiomas
    "cat": {"fr": "chat", "es": "gato", "de": "Katze"},  # Traducciones de "cat" a diferentes idiomas
    "dog": {"fr": "chien", "es": "perro", "de": "Hund"},  # Traducciones de "dog" a diferentes idiomas
    "apple": {"fr": "pomme", "es": "manzana", "de": "Apfel"}  # Traducciones de "apple" a diferentes idiomas
}

# Función para traducir una palabra a un idioma aleatorio
def traducir_palabra(palabra, idioma_destino):
    if palabra in diccionario:  # Verifica si la palabra está en el diccionario
        if idioma_destino in diccionario[palabra]:  # Verifica si el idioma destino está disponible para la palabra
            return diccionario[palabra][idioma_destino]  # Retorna la traducción de la palabra al idioma destino
    return "No se encontró traducción"  # Retorna un mensaje si la traducción no está disponible

# Función para traducir una oración a un idioma aleatorio
def traducir_oracion(oracion, idioma_destino):
    palabras = oracion.split()  # Divide la oración en palabras
    traduccion = []
    for palabra in palabras:
        traduccion.append(traducir_palabra(palabra, idioma_destino))  # Traduce cada palabra de la oración
    return ' '.join(traduccion)  # Une las palabras traducidas en una sola cadena

# Ejemplo de uso
oracion = "hello cat"  # Oración original
idioma_destino = random.choice(["fr", "es", "de"])  # Selecciona un idioma de destino aleatorio
print(f"Oración original: {oracion}")
print(f"Traducción al idioma {idioma_destino}: {traducir_oracion(oracion, idioma_destino)}")  # Imprime la traducción de la oración al idioma destino
