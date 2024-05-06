#Victor Eduardo Aleman Padilla 21310193


class ClasificadorBayesiano:
    def __init__(self, clases):
        # Inicializa el clasificador con una lista de clases posibles
        self.clases = clases
        # Inicializa los contadores de palabras por clase y el contador total de palabras
        self.contadores_clase = {clase: {} for clase in clases}  # Inicializa un diccionario para contar las palabras por clase
        self.contador_total_clase = {clase: 0 for clase in clases}  # Inicializa un diccionario para contar el total de palabras por clase
        self.contador_total = 0  # Inicializa el contador total de palabras

    def entrenar(self, datos, etiquetas):
        # Entrena el clasificador con los datos de entrenamiento y sus etiquetas
        for i in range(len(datos)):
            clase = etiquetas[i]
            # Separa el documento en palabras y cuenta las ocurrencias de cada palabra por clase
            for palabra in datos[i].split():
                self.contadores_clase[clase][palabra] = self.contadores_clase[clase].get(palabra, 0) + 1  # Incrementa el contador de la palabra en la clase actual
                self.contador_total_clase[clase] += 1  # Incrementa el contador total de palabras en la clase actual
                self.contador_total += 1  # Incrementa el contador total de palabras

    def probabilidad_palabra_dada_clase(self, palabra, clase, alfa=1):
        # Calcula la probabilidad de una palabra dada una clase utilizando suavizado de Laplace (alfa)
        if palabra in self.contadores_clase[clase]:
            return (self.contadores_clase[clase][palabra] + alfa) / (self.contador_total_clase[clase] + alfa * self.contador_total)
        else:
            return alfa / (self.contador_total_clase[clase] + alfa * self.contador_total)

    def probabilidad_clase_dada_documento(self, documento, clase):
        # Calcula la probabilidad de una clase dada un documento multiplicando las probabilidades de todas las palabras en el documento
        probabilidad = 1
        for palabra in documento.split():
            probabilidad *= self.probabilidad_palabra_dada_clase(palabra, clase)
        return probabilidad

    def predecir(self, documento):
        # Predice la clase de un documento calculando las probabilidades de todas las clases y seleccionando la clase con la mayor probabilidad
        mejor_clase = None
        mejor_probabilidad = -1
        for clase in self.clases:
            probabilidad_clase = self.probabilidad_clase_dada_documento(documento, clase)
            if probabilidad_clase > mejor_probabilidad:
                mejor_clase = clase
                mejor_probabilidad = probabilidad_clase
        return mejor_clase

# Ejemplo de uso
if __name__ == "__main__":
    # Datos de entrenamiento
    datos_entrenamiento = [
        "comprar pastillas para adelgazar",
        "oferta exclusiva, ¡gana un millón de dólares!",
        "reunión mañana por la mañana",
        "última llamada para actualizar su información de cuenta",
    ]
    etiquetas_entrenamiento = ["spam", "spam", "no_spam", "no_spam"]

    # Crear y entrenar el clasificador
    clasificador = ClasificadorBayesiano(["spam", "no_spam"])
    clasificador.entrenar(datos_entrenamiento, etiquetas_entrenamiento)

    # Documento de prueba
    documento_prueba = "comprar pastillas mañana por la mañana"

    # Predecir la clase del documento de prueba
    clase_predicha = clasificador.predecir(documento_prueba)
    print("El documento se clasifica como:", clase_predicha)
