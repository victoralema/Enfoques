#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importamos NumPy para realizar cálculos numéricos
import matplotlib.pyplot as plt  # Importamos Matplotlib para visualizar los datos

# Función para aplicar un filtro de suavizado a una serie de datos
def filtro_suavizado(datos, ventana):
    # Creamos una ventana de promedio móvil
    filtro = np.ones(ventana) / ventana  # Creamos un filtro que consiste en una ventana de tamaño "ventana" con valores de 1/ventana
    # Aplicamos el filtro de suavizado utilizando la función de convolución de NumPy
    datos_suavizados = np.convolve(datos, filtro, mode='valid')  # Aplicamos la convolución entre los datos y el filtro
    return datos_suavizados  # Retornamos los datos suavizados

# Generamos datos de ejemplo (una señal con ruido)
x = np.linspace(0, 10, 100)  # Creamos 100 puntos equiespaciados en el rango [0, 10]
y = np.sin(x) + np.random.normal(0, 0.1, 100)  # Generamos una señal sinusoidal con ruido

# Aplicamos el filtro de suavizado a los datos
ventana = 5  # Tamaño de la ventana del filtro de suavizado
y_suavizado = filtro_suavizado(y, ventana)  # Llamamos a la función de suavizado con los datos y el tamaño de la ventana

# Graficamos los datos originales y suavizados
plt.plot(x, y, label='Datos originales')  # Graficamos los datos originales
plt.plot(x[ventana-1:], y_suavizado, label='Datos suavizados', color='red', linewidth=2)  # Graficamos los datos suavizados
plt.title('Filtro de Suavizado')  # Título del gráfico
plt.xlabel('Tiempo')  # Etiqueta del eje x
plt.ylabel('Valor')  # Etiqueta del eje y
plt.legend()  # Agregamos leyenda al gráfico
plt.grid(True)  # Activamos la cuadrícula en el gráfico
plt.show()  # Mostramos el gráfico
