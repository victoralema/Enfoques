#Victor Eduardo Aleman Padilla 21310193

import pymunk  # Importa la biblioteca de simulación física
import numpy as np  # Importa la biblioteca numpy para operaciones numéricas

class Agente:
    def __init__(self, espacio, masa, radio, posicion, fuerza):
        # Constructor de la clase Agente
        self.masa = masa  # Define la masa del agente
        self.radio = radio  # Define el radio del agente
        self.cuerpo = pymunk.Body(masa, pymunk.moment_for_circle(masa, 0, radio))  # Crea el cuerpo del agente
        self.cuerpo.position = posicion  # Establece la posición inicial del agente
        self.forma = pymunk.Circle(self.cuerpo, radio)  # Crea la forma del agente
        self.espacio = espacio  # Establece el espacio en el que está el agente
        self.espacio.add(self.cuerpo, self.forma)  # Añade el cuerpo y la forma al espacio de simulación
        self.fuerza = fuerza  # Establece la fuerza del agente

    def mover(self):
        # Método para mover el agente aplicando una fuerza
        self.cuerpo.apply_force_at_local_point(self.fuerza.tolist(), (0, 0))  # Aplica la fuerza al agente en un punto local

class Entorno:
    def __init__(self):
        # Constructor de la clase Entorno
        self.espacio = pymunk.Space()  # Crea un espacio de simulación
        self.agentes = []  # Inicializa una lista para almacenar los agentes

    def agregar_agente(self, agente):
        # Método para agregar un agente al entorno
        self.agentes.append(agente)  # Añade el agente a la lista de agentes

    def paso_simulacion(self, fuerzas):
        # Método para avanzar un paso en la simulación
        for agente, fuerza in zip(self.agentes, fuerzas):
            # Itera sobre los agentes y las fuerzas proporcionadas
            agente.fuerza = fuerza  # Actualiza la fuerza del agente
            agente.mover()  # Hace que cada agente ejecute el método mover con la fuerza correspondiente
        self.espacio.step(1/60)  # Avanza la simulación un paso de tiempo

if __name__ == "__main__":
    entorno = Entorno()  # Crea un nuevo entorno de simulación
    fuerzas = [np.array((100, 0)), np.array((0, 100))]  # Define las fuerzas que se aplicarán a los agentes
    agente1 = Agente(entorno.espacio, 1, 10, (50, 50), fuerzas[0])  # Crea un agente en el entorno con ciertas características y la primera fuerza
    agente2 = Agente(entorno.espacio, 1, 10, (100, 100), fuerzas[1])  # Crea otro agente en el entorno con ciertas características y la segunda fuerza
    entorno.agregar_agente(agente1)  # Agrega el primer agente al entorno
    entorno.agregar_agente(agente2)  # Agrega el segundo agente al entorno

    # Simulación
    for i in range(1000):
        entorno.paso_simulacion(fuerzas)  # Avanza un paso en la simulación con las fuerzas proporcionadas
