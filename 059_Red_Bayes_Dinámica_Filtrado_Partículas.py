#Victor Eduardo Aleman Padilla 21310193

import numpy as np  # Importa la librería NumPy para operaciones matemáticas eficientes en arreglos

class ParticleFilter:
    def __init__(self, num_particles, initial_state, transition_model, observation_model, measurement_noise):
        # Inicializa el filtro de partículas con un número dado de partículas, estado inicial, modelos de transición y observación, y ruido de medición
        self.num_particles = num_particles  # Número de partículas
        self.particles = np.random.normal(initial_state, 1, size=(num_particles, len(initial_state)))  # Genera partículas aleatorias alrededor del estado inicial
        self.transition_model = transition_model  # Modelo de transición
        self.observation_model = observation_model  # Modelo de observación
        self.measurement_noise = measurement_noise  # Ruido de medición

    def predict(self, control_input):
        # Predice el estado futuro de las partículas según el modelo de transición y el control de entrada
        self.particles = self.transition_model(self.particles, control_input)  # Aplica el modelo de transición

    def update(self, measurement):
        # Actualiza las ponderaciones de las partículas basadas en la observación actual
        weights = self.observation_model(self.particles, measurement, self.measurement_noise)  # Calcula las ponderaciones
        weights += 1.e-300  # Evita la división por cero agregando un pequeño valor epsilon
        weights /= np.sum(weights)  # Normaliza las ponderaciones para obtener una distribución de probabilidad
        indices = np.random.choice(np.arange(self.num_particles), size=self.num_particles, replace=True, p=weights)  # Realiza el remuestreo ponderado
        self.particles = self.particles[indices]  # Actualiza las partículas seleccionadas

# Ejemplo de uso:
if __name__ == "__main__":
    # Define los modelos de transición y observación
    def transition_model(particles, control_input):
        # Ejemplo: caminata aleatoria
        return particles + np.random.normal(0, 0.1, size=particles.shape)

    def observation_model(particles, measurement, measurement_noise):
        # Ejemplo: verosimilitud gaussiana
        return np.exp(-0.5 * np.sum((particles - measurement) ** 2 / measurement_noise ** 2, axis=1))

    num_particles = 1000  # Número de partículas
    initial_state = np.array([0.0, 0.0])  # Estado inicial
    measurement_noise = 0.1  # Ruido de medición

    pf = ParticleFilter(num_particles, initial_state, transition_model, observation_model, measurement_noise)  # Inicializa el filtro de partículas

    # Simula movimientos y actualizaciones de medición
    for t in range(10):
        control_input = np.array([0.1, 0.1])  # Entrada de control de ejemplo
        pf.predict(control_input)  # Predice el siguiente estado de las partículas

        true_state = np.array([t * 0.1, t * 0.1])  # Estado verdadero (para simulación)
        measurement = true_state + np.random.normal(0, measurement_noise, size=2)  # Medición simulada

        pf.update(measurement)  # Actualiza el filtro de partículas con la nueva medición

        print("Time step:", t)
        print("Estimated state:", np.mean(pf.particles, axis=0))  # Imprime el estado estimado como la media de las partículas
        print()
