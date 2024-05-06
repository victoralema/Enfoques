#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importa la librería NumPy para operaciones matemáticas eficientes en arreglos

class KalmanFilter:
    def __init__(self, A, B, H, Q, R, P, x0):
        self.A = A  # Matriz de transición de estado
        self.B = B  # Matriz de control
        self.H = H  # Matriz de observación
        self.Q = Q  # Covarianza del ruido de proceso
        self.R = R  # Covarianza del ruido de medición
        self.P = P  # Covarianza del error de estimación
        self.x = x0 # Estimación inicial del estado

    def predict(self, u=0):
        # Predice el próximo estado
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)  # Predicción del estado
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  # Predicción del error de estimación

    def update(self, z):
        # Actualiza el Filtro de Kalman utilizando una nueva medición
        y = z - np.dot(self.H, self.x)  # Residuo de la medición
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R  # Covarianza residual
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Ganancia de Kalman
        self.x = self.x + np.dot(K, y)  # Actualización del estado
        I = np.eye(self.P.shape[0])  # Matriz identidad del tamaño de P
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)  # Actualización de la covarianza del error de estimación

# Ejemplo de uso:
if __name__ == "__main__":
    dt = 1.0  # Paso de tiempo

    # Define las matrices del sistema
    A = np.array([[1, dt], [0, 1]])  # Matriz de transición de estado
    B = np.array([[0.5*dt**2], [dt]])  # Matriz de control
    H = np.array([[1, 0]])  # Matriz de observación
    Q = np.array([[0.1, 0], [0, 0.1]])  # Covarianza del ruido de proceso
    R = np.array([[1]])  # Covarianza del ruido de medición
    P = np.array([[1, 0], [0, 1]])  # Covarianza del error de estimación
    x0 = np.array([[0], [0]])  # Estimación inicial del estado

    kf = KalmanFilter(A, B, H, Q, R, P, x0)  # Inicializa el filtro de Kalman

    # Genera algunas mediciones ruidosas
    measurements = [1, 2, 3, 4, 5]

    for z in measurements:
        kf.predict()  # Predice el próximo estado
        kf.update(np.array([[z]]))  # Actualiza el filtro con la nueva medición
        print("Measurement:", z)  # Imprime la medición actual
        print("Predicted state:", kf.x)  # Imprime el estado predicho
        print("Estimated error covariance:", kf.P)  # Imprime la covarianza del error de estimación
        print()

