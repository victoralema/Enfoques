#Victor Eduardo Aleman Padilla 21310193
import random  # Importamos la biblioteca random para generar números aleatorios
from collections import defaultdict  # Importamos defaultdict de la biblioteca collections para crear un diccionario con valores predeterminados

class LanguageModel:
    def __init__(self, n=2):
        # Inicializamos el modelo de lenguaje con un valor n dado para los n-gramas
        self.n = n  # Número de palabras en el n-grama
        self.ngrams = defaultdict(lambda: defaultdict(int))  # Creamos un diccionario anidado para almacenar los recuentos de n-gramas
        self.contexts = defaultdict(int)  # Creamos un diccionario para almacenar los recuentos de contextos de n-1 palabras

    def train(self, corpus):
        # Entrenamos el modelo de lenguaje con un corpus de texto dado
        tokens = corpus.split()  # Dividimos el corpus en tokens (palabras)
        for i in range(len(tokens) - self.n + 1):  # Iteramos sobre los índices en el rango del tamaño del n-grama en el corpus
            ngram = tuple(tokens[i:i+self.n])  # Creamos un n-grama como una tupla de palabras
            context = tuple(tokens[i:i+self.n-1])  # Creamos el contexto como una tupla de palabras
            self.ngrams[context][ngram[-1]] += 1  # Incrementamos el contador de ocurrencias del siguiente término del n-grama dado el contexto
            self.contexts[context] += 1  # Incrementamos el contador de ocurrencias del contexto de n-1 palabras

    def generate_next_word(self, context):
        # Generamos la siguiente palabra basada en el contexto dado
        candidates = self.ngrams[context]  # Obtenemos las posibles siguientes palabras dado el contexto
        total = sum(candidates.values())  # Calculamos el número total de ocurrencias de todas las posibles siguientes palabras
        rand = random.uniform(0, total)  # Elegimos un número aleatorio en el rango de [0, total) para seleccionar una palabra basada en probabilidades
        cumulative = 0  # Inicializamos un contador acumulativo
        for word, count in candidates.items():  # Iteramos sobre las posibles siguientes palabras y sus ocurrencias
            cumulative += count  # Incrementamos el contador acumulativo con el número de ocurrencias de la palabra actual
            if rand < cumulative:  # Si el número aleatorio está dentro del rango de las ocurrencias acumuladas
                return word  # Devolvemos la palabra actual como la siguiente palabra
        return None  # Devolvemos None si no se encuentra ninguna palabra adecuada

    def generate_text(self, length=10, seed=None):
        # Generamos texto usando el modelo de lenguaje
        if seed is not None and len(seed) >= self.n-1:  # Si se proporciona una semilla inicial y tiene al menos n-1 palabras
            context = tuple(seed[-(self.n-1):])  # Utilizamos las últimas n-1 palabras de la semilla como contexto inicial
        else:  # Si no se proporciona una semilla inicial o es demasiado corta
            context = random.choice(list(self.contexts.keys()))  # Elegimos un contexto aleatorio de n-1 palabras

        text = list(context)  # Convertimos el contexto en una lista de palabras
        while len(text) < length:  # Continuamos generando palabras hasta alcanzar la longitud deseada
            word = self.generate_next_word(context)  # Generamos la siguiente palabra dado el contexto actual
            if word is None:  # Si no se puede encontrar una siguiente palabra
                break  # Salimos del bucle
            text.append(word)  # Añadimos la palabra al texto generado
            context = context[1:] + (word,)  # Actualizamos el contexto moviendo una palabra hacia la izquierda y añadiendo la nueva palabra
        return ' '.join(text)  # Devolvemos el texto generado como una cadena de palabras separadas por espacios

# Ejemplo de uso
corpus = "El gato está en la casa. La casa es grande. El perro está afuera."  # Definimos un corpus de texto de ejemplo
lm = LanguageModel(n=2)  # Creamos una instancia del modelo de lenguaje con bigramas
lm.train(corpus)  # Entrenamos el modelo con el corpus de texto

# Generamos texto utilizando el modelo de lenguaje
generated_text = lm.generate_text(length=10, seed="La casa")  # Generamos texto de longitud 10 con la semilla "La casa"
print("Texto generado:", generated_text)  # Imprimimos el texto generado
