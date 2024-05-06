#Victor Eduardo Aleman Padilla 21310193
import numpy as np  # Importa NumPy para operaciones numéricas
import matplotlib.pyplot as plt  # Importa Matplotlib para visualización

class PIDController:
    def __init__(self, Kp):
        self.Kp = Kp  # Establece el parámetro de ganancia proporcional (Kp) del controlador PID

    def compute_control_output(self, error):
        # Calcula la salida de control proporcional al error
        return self.Kp * error

class System:
    def __init__(self, initial_position, max_velocity):
        self.position = initial_position  # Establece la posición inicial del sistema
        self.max_velocity = max_velocity  # Establece la velocidad máxima permitida para el sistema

    def update(self, control_input):
        # Simula el comportamiento dinámico del sistema
        self.position += control_input
        # Limita la velocidad máxima del sistema
        self.position = np.clip(self.position, -self.max_velocity, self.max_velocity)

if __name__ == "__main__":
    # Parámetros del sistema
    initial_position = 0  # Posición inicial del sistema
    desired_position = 10  # Posición deseada a la que el sistema debe llegar
    max_velocity = 1  # Velocidad máxima permitida para el sistema

    # Parámetros del controlador PID
    Kp = 0.5  # Ganancia proporcional del controlador P
    controller = PIDController(Kp)  # Crea una instancia del controlador PID con la ganancia proporcional especificada

    # Inicializa el sistema
    system = System(initial_position, max_velocity)  # Crea una instancia del sistema con la posición inicial y la velocidad máxima

    # Simulación del sistema
    num_steps = 100  # Número de pasos de simulación
    time = np.arange(num_steps)  # Crea un arreglo de tiempo
    position_history = []  # Lista para almacenar el historial de posiciones del sistema

    for _ in time:
        # Calcula el error entre la posición deseada y la posición actual del sistema
        error = desired_position - system.position
        # Calcula la salida de control utilizando el controlador PID y el error actual
        control_output = controller.compute_control_output(error)
        # Actualiza el sistema con la salida de control
        system.update(control_output)
        # Guarda la posición actual del sistema en el historial de posiciones
        position_history.append(system.position)

    # Visualiza la respuesta del sistema (historial de posiciones)
    plt.plot(time, position_history)  # Traza el historial de posiciones
    plt.xlabel('Tiempo')  # Etiqueta del eje x
    plt.ylabel('Posición')  # Etiqueta del eje y
    plt.title('Respuesta del Sistema con Controlador P')  # Título del gráfico
    plt.grid(True)  # Muestra la cuadrícula en el gráfico
    plt.show()  # Muestra el gráfico
