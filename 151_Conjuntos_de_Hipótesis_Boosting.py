#Victor Eduardo Aleman Padilla 21310193
import numpy as np

# Definición de la clase de hipótesis
class Hipotesis:
    def __init__(self, peso=1):
        self.w = None  # Parámetros de la hipótesis
        self.bias = None
        self.peso = peso  # Peso inicial de la hipótesis

    def entrenar(self, datos_entrenamiento, etiquetas, pesos_instancias):
        # Entrenar la hipótesis utilizando un clasificador simple, como un perceptrón
        num_caracteristicas = len(datos_entrenamiento[0])  # Número de características
        self.w = np.random.rand(num_caracteristicas)  # Inicialización de pesos aleatorios
        self.bias = np.random.rand()  # Inicialización de sesgo aleatorio

        # Ajuste de pesos basado en la precisión
        errores = (np.sign(np.dot(datos_entrenamiento, self.w) + self.bias) != etiquetas) * 1  # Errores de predicción
        errores_pesados = errores * pesos_instancias  # Multiplicación de errores por pesos de instancias
        error_total = np.sum(errores_pesados)  # Cálculo del error total
        if error_total > 0.5:
            error_total = 1 - error_total
            self.peso = 0.5 * np.log((1 - error_total) / error_total)  # Cálculo del peso de la hipótesis
        else:
            self.peso = 0.5 * np.log((1 - error_total) / error_total)  # Cálculo del peso de la hipótesis

    def evaluar(self, instancia):
        # Evaluar la hipótesis en una instancia y devolver la predicción
        return np.sign(np.dot(instancia, self.w) + self.bias)  # Predicción de la hipótesis

# Definición de la clase del conjunto de hipótesis
class ConjuntoHipotesis:
    def __init__(self, T):
        self.T = T  # Número de hipótesis
        self.hipotesis = []  # Lista para almacenar las hipótesis

    def entrenar(self, datos_entrenamiento, etiquetas, pesos_instancias):
        # Entrenar el conjunto de hipótesis utilizando el algoritmo de Boosting
        num_instancias = len(datos_entrenamiento)  # Número de instancias
        pesos_instancias = np.array(pesos_instancias)  # Conversión de lista a array NumPy

        for _ in range(self.T):
            # Crear una nueva hipótesis
            nueva_hipotesis = Hipotesis()
            nueva_hipotesis.entrenar(datos_entrenamiento, etiquetas, pesos_instancias)

            # Actualizar pesos de las instancias
            predicciones = np.array([nueva_hipotesis.evaluar(instancia) for instancia in datos_entrenamiento])
            pesos_instancias *= np.exp(-nueva_hipotesis.peso * etiquetas * predicciones)
            pesos_instancias /= np.sum(pesos_instancias)

            # Agregar la hipótesis entrenada al conjunto
            self.hipotesis.append(nueva_hipotesis)

    def predecir(self, instancia):
        # Realizar predicciones utilizando el conjunto de hipótesis entrenado
        predicciones = [hipotesis.evaluar(instancia) * hipotesis.peso for hipotesis in self.hipotesis]
        return np.sign(np.sum(predicciones))  # Predicción final del conjunto de hipótesis

# Algoritmo de Boosting para conjuntos de hipótesis
def boosting(datos_entrenamiento, etiquetas, T):
    conjunto_hipotesis = ConjuntoHipotesis(T)  # Creación de un conjunto de hipótesis
    num_instancias = len(datos_entrenamiento)  # Número de instancias
    pesos_instancias = np.ones(num_instancias) / num_instancias  # Pesos iniciales iguales
    conjunto_hipotesis.entrenar(datos_entrenamiento, etiquetas, pesos_instancias)  # Entrenamiento del conjunto de hipótesis
    return conjunto_hipotesis  # Devolver el conjunto de hipótesis entrenado

# Ejemplo de uso del algoritmo de Boosting
datos_entrenamiento = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])  # Datos de entrenamiento
etiquetas = np.array([-1, 1, 1, 1])  # Clasificación binaria
T = 10  # Número de hipótesis a utilizar
modelo_boosting = boosting(datos_entrenamiento, etiquetas, T)  # Entrenamiento del modelo

# Ejemplo de predicción
instancia_nueva = np.array([0, 0])  # Nueva instancia a predecir
prediccion = modelo_boosting.predecir(instancia_nueva)  # Predicción del modelo
print("Predicción:", prediccion)  # Imprimir la predicción
