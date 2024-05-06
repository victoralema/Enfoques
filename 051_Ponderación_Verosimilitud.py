#Victor Eduardo Aleman Padilla 21310193

import numpy as np

# Inicialización de probabilidades iniciales uniformes para los números del dado (1 al 6)
probabilidades = np.ones(6) / 6

# Definir la función de actualización de probabilidades usando ponderación de verosimilitud
def actualizar_probabilidades(resultados_observados, probabilidades):
    verosimilitudes = np.zeros(6)
    for i in range(6):
        verosimilitud = 1.0
        for resultado in resultados_observados:
            if resultado == i + 1:
                verosimilitud *= 0.6  # Ponderación de verosimilitud: multiplicar por 0.6 si coincide
            else:
                verosimilitud *= 0.4  # Ponderación de verosimilitud: multiplicar por 0.4 si no coincide
        verosimilitudes[i] = verosimilitud

    # Normalizar las nuevas probabilidades
    nuevas_probabilidades = verosimilitudes * probabilidades
    nuevas_probabilidades /= np.sum(nuevas_probabilidades)
    return nuevas_probabilidades

# Simular el lanzamiento del dado y actualizar las probabilidades
lanzamientos = [3, 5, 2, 3, 6]  # Resultados observados

for lanzamiento in lanzamientos:
    probabilidades = actualizar_probabilidades([lanzamiento], probabilidades)

# Mostrar las probabilidades actualizadas
for i in range(6):
    print(f"Probabilidad de obtener {i + 1}: {probabilidades[i]:.4f}")
