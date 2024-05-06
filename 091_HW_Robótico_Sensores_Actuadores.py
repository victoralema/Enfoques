#Victor Eduardo Aleman Padilla 21310193
# Importamos el módulo random para generar valores aleatorios
import random

# Definimos una clase para el robot
class Robot:
    # Constructor de la clase
    def __init__(self):
        self.position = 0  # Inicializamos la posición del robot en 0

    # Método para simular la medición de distancia con un sensor
    def measure_distance(self):
        # Generamos un valor aleatorio entre 0 y 100 para simular la medición
        return random.randint(0, 100)

    # Método para mover el robot hacia adelante
    def move_forward(self):
        self.position += 1  # Incrementamos la posición en 1 para mover hacia adelante

    # Método para mover el robot hacia atrás
    def move_backward(self):
        self.position -= 1  # Decrementamos la posición en 1 para mover hacia atrás

# Creamos un objeto para representar al robot
robot = Robot()

# Realizamos una serie de acciones basadas en mediciones
for _ in range(10):  # Repetimos el proceso 10 veces
    distance = robot.measure_distance()  # Medimos la distancia
    print("Distancia medida:", distance)  # Imprimimos la distancia medida

    # Tomamos decisiones basadas en la distancia medida
    if distance < 30:  # Si la distancia es menor que 30
        robot.move_forward()  # Movemos el robot hacia adelante
        print("Moviendo hacia adelante")  # Imprimimos un mensaje indicando el movimiento
    elif distance > 70:  # Si la distancia es mayor que 70
        robot.move_backward()  # Movemos el robot hacia atrás
        print("Moviendo hacia atrás")  # Imprimimos un mensaje indicando el movimiento
    else:  # Si la distancia está entre 30 y 70
        print("Permaneciendo en la misma posición")  # Imprimimos un mensaje indicando la inacción

# Imprimimos la posición final del robot después de completar todas las acciones
print("Posición final del robot:", robot.position)
