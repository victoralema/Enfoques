#Victor Eduardo Aleman Padilla 21310193

# Importar la librería numpy para operaciones numéricas y matplotlib.pyplot para visualización
import numpy as np
import matplotlib.pyplot as plt

# Definir una función para demostrar la generación de muestras de una distribución
def distribution_demo(mean, std_dev, sample_size):
    # Generar muestras aleatorias de una distribución normal con la media 'mean' y desviación estándar 'std_dev'
    samples = np.random.normal(mean, std_dev, sample_size)
    
    # Calcular la media y la desviación estándar de las muestras generadas
    sample_mean = np.mean(samples)
    sample_std_dev = np.std(samples)
    
    # Imprimir las muestras generadas y las estadísticas calculadas
    print(f"Muestras generadas de una distribución normal con media {mean} y desviación estándar {std_dev}:")
    print(samples)
    print("\nEstadísticas:")
    print(f"Media muestral: {sample_mean}")
    print(f"Desviación estándar muestral: {sample_std_dev}")
    
    # Visualización de un histograma de las muestras generadas
    plt.hist(samples, bins=30, density=True, alpha=0.6, color='g')
    plt.title('Distribución de probabilidad')
    plt.xlabel('Valor')
    plt.ylabel('Frecuencia')
    plt.show()

# Parámetros de la distribución
media = 0
desviacion_estandar = 1
tamaño_muestra = 1000

# Llamar a la función 'distribution_demo' con los parámetros especificados y mostrar resultados
distribution_demo(media, desviacion_estandar, tamaño_muestra)
