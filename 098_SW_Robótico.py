#Victor Eduardo Aleman Padilla 21310193
import random  # Importamos el módulo random para generar números aleatorios

class Robot:
    def __init__(self, position):
        self.position = position  # Inicializamos la posición del robot

    def move(self):
        # Simulamos el movimiento del robot
        if random.random() < 0.8:  # Probabilidad de 80% de moverse hacia adelante
            self.position += 1
        else:
            self.position -= 1  # Probabilidad de 20% de moverse hacia atrás

class Sensor:
    def __init__(self, robot_position):
        self.robot_position = robot_position  # Inicializamos la posición del robot para el sensor

    def sense(self):
        # Simulamos la lectura del sensor con un poco de ruido
        return self.robot_position + random.randint(-1, 1)  # Añadimos un ruido de +/- 1 a la posición del robot

def main():
    robot = Robot(position=0)  # Creamos una instancia del robot en la posición inicial 0
    sensor = Sensor(robot_position=robot.position)  # Creamos una instancia del sensor con la posición inicial del robot

    for _ in range(10):  # Realizamos 10 iteraciones para simular el movimiento del robot
        robot.move()  # Movemos el robot
        sensed_position = sensor.sense()  # Obtenemos una lectura del sensor
        print(f"Robot's actual position: {robot.position}, Sensor reading: {sensed_position}")  # Imprimimos la posición real del robot y la lectura del sensor

if __name__ == "__main__":
    main()  # Llamamos a la función main si el script es ejecutado directamente
