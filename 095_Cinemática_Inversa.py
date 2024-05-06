#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importa NumPy para operaciones numéricas
import matplotlib.pyplot as plt  # Importa Matplotlib para visualización

class RobotArm:
    def __init__(self, link1_length, link2_length):
        self.link1_length = link1_length  # Longitud del primer eslabón
        self.link2_length = link2_length  # Longitud del segundo eslabón

    def inverse_kinematics(self, x_target, y_target):
        # Calcula la distancia desde el origen hasta el objetivo
        distance_to_target = np.sqrt(x_target**2 + y_target**2)

        # Calcula el ángulo entre el objetivo y el segundo eslabón usando la ley de los cosenos
        theta2 = np.arccos((self.link1_length**2 + self.link2_length**2 - distance_to_target**2) /
                           (2 * self.link1_length * self.link2_length))

        # Calcula el ángulo de la primera articulación usando la ley de los cosenos
        alpha = np.arctan2(y_target, x_target)
        beta = np.arccos((self.link1_length**2 + distance_to_target**2 - self.link2_length**2) /
                         (2 * self.link1_length * distance_to_target))
        theta1 = alpha - beta

        return theta1, theta2

if __name__ == "__main__":
    # Longitudes de los eslabones del brazo del robot
    link1_length = 3
    link2_length = 2

    # Crea el brazo del robot
    robot_arm = RobotArm(link1_length, link2_length)

    # Coordenadas del objetivo
    x_target = 4
    y_target = 1

    # Calcula la cinemática inversa para alcanzar el objetivo
    theta1, theta2 = robot_arm.inverse_kinematics(x_target, y_target)

    # Imprime los ángulos de las articulaciones
    print("Ángulo de la primera articulación (theta1):", np.degrees(theta1))
    print("Ángulo de la segunda articulación (theta2):", np.degrees(theta2))

    # Visualiza el brazo del robot y el objetivo
    fig, ax = plt.subplots()
    # Dibuja el brazo del robot usando los ángulos de las articulaciones calculados
    ax.plot([0, robot_arm.link1_length * np.cos(theta1),
             robot_arm.link1_length * np.cos(theta1) + robot_arm.link2_length * np.cos(theta1 + theta2)],
            [0, robot_arm.link1_length * np.sin(theta1),
             robot_arm.link1_length * np.sin(theta1) + robot_arm.link2_length * np.sin(theta1 + theta2)], 'bo-')
    ax.plot(x_target, y_target, 'ro')  # Marca el objetivo en rojo
    ax.set_xlim(-1, 7)  # Límites del eje x en el gráfico
    ax.set_ylim(-1, 5)  # Límites del eje y en el gráfico
    ax.set_aspect('equal')  # Asigna la misma escala a ambos ejes para evitar distorsiones
    plt.xlabel('X')  # Etiqueta del eje x
    plt.ylabel('Y')  # Etiqueta del eje y
    plt.title('Cinemática Inversa')  # Título del gráfico
    plt.grid(True)  # Muestra la cuadrícula en el gráfico
    plt.show()  # Muestra el gráfico
