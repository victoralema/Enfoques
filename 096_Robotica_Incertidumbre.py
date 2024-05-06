#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importa NumPy para operaciones numéricas
import matplotlib.pyplot as plt  # Importa Matplotlib para visualización

class Robot:
    def __init__(self, initial_pose, motion_noise, measurement_noise):
        self.pose = np.array(initial_pose, dtype=np.float64)  # Inicializa la pose (posición) del robot como float64
        self.motion_noise = motion_noise  # Define el nivel de ruido de movimiento
        self.measurement_noise = measurement_noise  # Define el nivel de ruido de medición

    def move(self, u):
        # Simula el movimiento del robot agregando ruido de movimiento
        self.pose += u + np.random.randn(2) * self.motion_noise.astype(np.float64)  # Asegura que el ruido de movimiento sea de tipo float64


    def sense(self):
        # Simula la medición de la posición del robot agregando ruido de medición
        return self.pose + np.random.randn(2) * self.measurement_noise

class EKF_Localization:
    def __init__(self, initial_pose, motion_noise, measurement_noise):
        self.belief = initial_pose  # Inicializa la creencia del robot sobre su posición
        self.motion_noise = motion_noise  # Define el nivel de ruido de movimiento
        self.measurement_noise = measurement_noise  # Define el nivel de ruido de medición

    def update(self, u, z):
        # Predicción de la nueva creencia utilizando el modelo de movimiento del robot
        self.belief += u
        # Actualización de la creencia basada en la medición utilizando el filtro de Kalman extendido
        K = np.dot(self.belief, self.belief) / (np.dot(self.belief, self.belief) + self.measurement_noise)
        self.belief += K * (z - self.belief)

if __name__ == "__main__":
    initial_pose = np.array([0, 0], dtype=np.float64)
    motion_noise = np.float64(0.1)
    measurement_noise = np.float64(0.1)
    robot = Robot(initial_pose, motion_noise, measurement_noise)  # Inicializa el robot
    ekf_localization = EKF_Localization(initial_pose, motion_noise, measurement_noise)  # Inicializa el filtro de Kalman extendido

    true_trajectory = [initial_pose]  # Inicializa la lista de la trayectoria verdadera del robot
    estimated_trajectory = [initial_pose]  # Inicializa la lista de la trayectoria estimada del robot

    for _ in range(20):
        # Movimiento verdadero del robot
        u = np.array([0.1, 0.1])  # Movimiento predeterminado del robot
        robot.move(u)  # Simula el movimiento del robot
        true_trajectory.append(robot.pose)  # Añade la posición actual del robot a la trayectoria verdadera

        # Medición de la posición del robot
        z = robot.sense()  # Simula la medición de la posición del robot

        # Actualización del filtro de Kalman extendido
        ekf_localization.update(u, z)  # Actualiza la estimación de la posición del robot
        estimated_trajectory.append(ekf_localization.belief)  # Añade la estimación actual de la posición del robot a la trayectoria estimada

    # Convierte las trayectorias a matrices numpy
    true_trajectory = np.array(true_trajectory)
    estimated_trajectory = np.array(estimated_trajectory)

    # Visualiza la trayectoria verdadera y la trayectoria estimada del robot
    plt.plot(true_trajectory[:, 0], true_trajectory[:, 1], label='Trayectoria verdadera')  # Traza la trayectoria verdadera
    plt.plot(estimated_trajectory[:, 0], estimated_trajectory[:, 1], label='Trayectoria estimada')  # Traza la trayectoria estimada
    plt.xlabel('X')  # Etiqueta del eje x
    plt.ylabel('Y')  # Etiqueta del eje y
    plt.title('Localización del Robot con Filtro de Kalman Extendido')  # Título del gráfico
    plt.legend()  # Muestra la leyenda
    plt.grid(True)  # Muestra la cuadrícula en el gráfico
    plt.show()  # Muestra el gráfico
