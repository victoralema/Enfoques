#Victor Eduardo Aleman Padilla 21310193
import numpy as np

# Definición de la clase de hipótesis
class Hipotesis:
    def __init__(self, num_caracteristicas):
        # Inicializar los parámetros de la hipótesis
        self.params = np.random.rand(num_caracteristicas + 1)  # +1 para el sesgo
    
    def evaluar(self, instancia):
        # Agregar un 1 al inicio de la instancia para representar el sesgo
        instancia_con_sesgo = np.insert(instancia, 0, 1)
        # Calcular la salida de la hipótesis utilizando la función sigmoide
        salida = self.sigmoid(np.dot(self.params, instancia_con_sesgo))
        return salida
    
    def sigmoid(self, x):
        # Función sigmoide para calcular la salida de la hipótesis
        return 1 / (1 + np.exp(-x))

# Algoritmo de Mejor Hipótesis Actual (MHA)
def mejor_hipotesis_actual(datos_entrenamiento, etiquetas, num_iteraciones, tasa_aprendizaje):
    num_instancias, num_caracteristicas = datos_entrenamiento.shape
    # Inicializar una hipótesis aleatoria
    hipotesis_actual = Hipotesis(num_caracteristicas)
    for _ in range(num_iteraciones):
        # Calcular el gradiente descendente para actualizar los parámetros de la hipótesis
        gradientes = np.zeros(num_caracteristicas + 1)
        for i in range(num_instancias):
            error = etiquetas[i] - hipotesis_actual.evaluar(datos_entrenamiento[i])
            instancia_con_sesgo = np.insert(datos_entrenamiento[i], 0, 1)
            gradientes += error * instancia_con_sesgo
        # Actualizar los parámetros de la hipótesis utilizando el gradiente descendente
        hipotesis_actual.params += tasa_aprendizaje * gradientes
    return hipotesis_actual

# Ejemplo de uso del algoritmo de Mejor Hipótesis Actual (MHA)
datos_entrenamiento = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
etiquetas = np.array([0, 1, 1, 1])  # Clasificación binaria
num_iteraciones = 1000  # Número de iteraciones de entrenamiento
tasa_aprendizaje = 0.1  # Tasa de aprendizaje
mejor_hipotesis = mejor_hipotesis_actual(datos_entrenamiento, etiquetas, num_iteraciones, tasa_aprendizaje)

# Ejemplo de predicción
instancia_nueva = np.array([0, 0])  # Nueva instancia a predecir
prediccion = mejor_hipotesis.evaluar(instancia_nueva)
print("Predicción:", prediccion)
