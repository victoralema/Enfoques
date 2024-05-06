#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importa la librería NumPy para operaciones numéricas
import matplotlib.pyplot as plt  # Importa la librería Matplotlib para visualización

class ConfigurationSpace:
    def __init__(self, width, height):
        self.width = width  # Ancho del espacio de configuración
        self.height = height  # Alto del espacio de configuración

    def is_valid_position(self, position):
        x, y = position
        # Verifica si la posición dada está dentro del espacio de configuración
        return 0 <= x < self.width and 0 <= y < self.height

class Robot:
    def __init__(self, start_position, configuration_space):
        self.position = start_position  # Posición inicial del robot
        self.configuration_space = configuration_space  # Espacio de configuración del robot

    def move(self, dx, dy):
        # Calcula la nueva posición sumando el desplazamiento a la posición actual
        new_position = np.array(self.position) + np.array([dx, dy])
        # Verifica si la nueva posición es válida
        if self.configuration_space.is_valid_position(new_position):
            # Actualiza la posición del robot si es válida
            self.position = new_position

if __name__ == "__main__":
    # Crear el espacio de configuración con ancho 10 y alto 10
    configuration_space = ConfigurationSpace(width=10, height=10)

    # Crear el robot en la posición inicial (5, 5)
    start_position = (5, 5)
    robot = Robot(start_position, configuration_space)

    # Mover el robot
    robot.move(1, 0)  # Mover hacia la derecha
    robot.move(0, -2) # Mover hacia abajo

    # Imprimir la posición final del robot
    print("Posición final del robot:", robot.position)

    # Visualizar el espacio de configuración y la posición final del robot
    plt.figure(figsize=(6, 6))
    # Marcar la posición final del robot en rojo ('ro')
    plt.plot(robot.position[0], robot.position[1], 'ro', label='Posición final del robot')
    plt.xlim(0, configuration_space.width)  # Límites en el eje x
    plt.ylim(0, configuration_space.height)  # Límites en el eje y
    plt.xlabel('X')  # Etiqueta del eje x
    plt.ylabel('Y')  # Etiqueta del eje y
    plt.title('Espacio de Configuración')  # Título del gráfico
    plt.grid(True)  # Mostrar cuadrícula
    plt.legend()  # Mostrar leyenda
    plt.show()  # Mostrar el gráfico
