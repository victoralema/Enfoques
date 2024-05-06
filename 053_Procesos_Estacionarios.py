#Victor Eduardo Aleman Padilla 21310193

import numpy as np
import matplotlib.pyplot as plt

# Función para simular un proceso estacionario (caminata aleatoria)
def simular_proceso_estacionario(num_pasos, media=0, desviacion=1):
    # Inicializamos la serie temporal con el valor inicial
    serie_temporal = [media]
    
    # Generamos los pasos siguientes del proceso
    for _ in range(1, num_pasos):
        # Generamos un nuevo valor aleatorio (ruido)
        ruido = np.random.normal(loc=media, scale=desviacion)
        
        # El siguiente valor en la serie es el valor anterior más el ruido
        nuevo_valor = serie_temporal[-1] + ruido
        
        # Agregamos el nuevo valor a la serie temporal
        serie_temporal.append(nuevo_valor)
    
    return serie_temporal

# Parámetros para la simulación
num_pasos = 100
media = 0
desviacion = 1

# Simulamos el proceso estacionario
serie_temporal = simular_proceso_estacionario(num_pasos, media, desviacion)

# Graficamos la serie temporal generada
plt.figure(figsize=(10, 6))
plt.plot(range(num_pasos), serie_temporal, marker='o', linestyle='-')
plt.xlabel('Tiempo')
plt.ylabel('Valor')
plt.title('Simulación de un Proceso Estacionario')
plt.grid(True)
plt.show()
