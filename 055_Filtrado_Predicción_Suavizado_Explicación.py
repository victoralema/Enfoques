#Victor Eduardo Aleman Padilla 21310193

from filterpy.kalman import KalmanFilter
import numpy as np

# Creamos un filtro de Kalman
kf = KalmanFilter(dim_x=2, dim_z=1)  # dim_x es la dimensión del estado, dim_z es la dimensión de la medición

# Definimos la matriz de transición del estado (A)
kf.F = np.array([[1., 1.],  # Transición del estado: x = x + dx, dx = constante
                  [0., 1.]]) # dx = dx

# Definimos la matriz de medición (H)
kf.H = np.array([[1., 0.]])  # Solo medimos la posición, no la velocidad

# Definimos la covarianza del ruido del proceso (Q) y del ruido de la medición (R)
kf.Q *= 0.001  # Ruido del proceso
kf.R *= 0.01   # Ruido de la medición

# Definimos el estado inicial y su covarianza
kf.x = np.array([0., 0.])  # Estado inicial: posición = 0, velocidad = 0
kf.P *= 10.  # Covarianza inicial del estado

# Generamos mediciones simuladas
measurements = [1, 2, 3, 4, 5]

# Filtramos las mediciones
filtered_states = []
for z in measurements:
    kf.predict()  # Predicción del siguiente estado
    kf.update(z)  # Actualización del estado con la nueva medición
    filtered_states.append(kf.x[0])  # Guardamos la posición estimada

print("Posiciones filtradas:", filtered_states)
