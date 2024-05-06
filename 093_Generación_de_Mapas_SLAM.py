#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importa NumPy para trabajar con matrices y operaciones numéricas

class SLAM:
    def __init__(self):
        self.landmarks = {}  # Almacena las coordenadas de los puntos de referencia
        self.robot_position = np.array([0, 0])  # Inicializa la posición del robot
        self.robot_trajectory = [self.robot_position]  # Almacena la trayectoria del robot

    def move_robot(self, distance, angle):
        # Calcula el desplazamiento en las coordenadas x e y
        dx = distance * np.cos(angle)
        dy = distance * np.sin(angle)

        # Actualiza la posición del robot
        self.robot_position[0] += dx
        self.robot_position[1] += dy

        # Registra la nueva posición en la trayectoria del robot
        self.robot_trajectory.append(self.robot_position.copy())

    def add_landmark(self, landmark_id, x, y):
        # Registra las coordenadas del punto de referencia en el diccionario
        self.landmarks[landmark_id] = np.array([x, y])

    def estimate_robot_position(self):
        # Estima la posición del robot como el promedio de la trayectoria
        return np.mean(self.robot_trajectory, axis=0)

if __name__ == "__main__":
    slam = SLAM()  # Crea una instancia del objeto SLAM

    # Mueve el robot y añade puntos de referencia
    slam.move_robot(1, np.pi/4)  # Mueve el robot 1 unidad en la dirección de 45 grados
    slam.add_landmark(1, 1, 1)   # Añade un punto de referencia con identificador 1 en la posición (1, 1)
    slam.move_robot(2, np.pi/3)  # Mueve el robot 2 unidades en la dirección de 60 grados
    slam.add_landmark(2, 2, 3)   # Añade un punto de referencia con identificador 2 en la posición (2, 3)
    slam.move_robot(1.5, np.pi/2) # Mueve el robot 1.5 unidades en la dirección de 90 grados
    slam.add_landmark(3, 0, 4)    # Añade un punto de referencia con identificador 3 en la posición (0, 4)

    # Estima la posición final del robot
    estimated_position = slam.estimate_robot_position()
    print("Posición estimada del robot:", estimated_position)  # Imprime la posición estimada del robot
