#Victor Eduardo Aleman Padilla 21310193
import random

# Datos de ejemplo (puedes cambiarlos según tus necesidades)
datos_completos = [3, 7, 2, 9, 5, 11, 6, 4, 8, 10]

# Función para introducir un porcentaje de datos incompletos
def introducir_datos_incompletos(datos, porcentaje_incompleto):
    cantidad_incompleta = int(len(datos) * porcentaje_incompleto)  # Calcula la cantidad de datos incompletos
    indices_incompletos = random.sample(range(len(datos)), cantidad_incompleta)  # Selecciona índices aleatorios para los datos incompletos
    datos_incompletos = datos.copy()  # Crea una copia de los datos originales
    for indice in indices_incompletos:  # Itera sobre los índices de los datos incompletos
        datos_incompletos[indice] = None  # Establece los valores correspondientes a None para indicar datos incompletos
    return datos_incompletos

# Función para recuperar los datos faltantes utilizando el método de la media
def recuperar_datos_media(datos):
    datos_sin_nones = [dato for dato in datos if dato is not None]  # Filtra los datos completos
    media = sum(datos_sin_nones) / len(datos_sin_nones)  # Calcula la media de los datos completos
    datos_recuperados = [dato if dato is not None else media for dato in datos]  # Reemplaza los valores faltantes por la media
    return datos_recuperados

# Función para recuperar los datos faltantes utilizando el método de Monte Carlo
# Función para recuperar los datos faltantes utilizando el método de Monte Carlo
def recuperar_datos_monte_carlo(datos, num_simulaciones):
    datos_recuperados = datos.copy()  # Crea una copia de los datos originales
    for i in range(len(datos)):  # Itera sobre los índices de los datos
        if datos[i] is None:  # Verifica si el dato en la posición actual es incompleto
            datos_completos = [dato for dato in datos if dato is not None]  # Filtra los datos completos
            simulaciones = [random.choice(datos_completos) for _ in range(num_simulaciones)]  # Realiza múltiples simulaciones, excluyendo los valores None
            datos_recuperados[i] = sum(simulaciones) / num_simulaciones  # Calcula el promedio de las simulaciones
    return datos_recuperados


# Ejemplo de uso
porcentaje_incompleto = 0.3  # Porcentaje de datos incompletos
datos_incompletos = introducir_datos_incompletos(datos_completos, porcentaje_incompleto)  # Introduce datos incompletos
print("Datos completos:", datos_completos)
print("Datos incompletos:", datos_incompletos)

# Recuperación de datos utilizando el método de la media
datos_recuperados_media = recuperar_datos_media(datos_incompletos)
print("Recuperación de datos utilizando el método de la media:", datos_recuperados_media)

# Recuperación de datos utilizando el método de Monte Carlo
num_simulaciones = 1000  # Número de simulaciones para el método de Monte Carlo
datos_recuperados_monte_carlo = recuperar_datos_monte_carlo(datos_incompletos, num_simulaciones)
print("Recuperación de datos utilizando el método de Monte Carlo:", datos_recuperados_monte_carlo)
