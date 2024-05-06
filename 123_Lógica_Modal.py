#Victor Eduardo Aleman Padilla 21310193

class AgenteModal:
    def __init__(self):
        self.creencias = set()  # Creamos un conjunto para almacenar las creencias del agente

    def agregar_creencia(self, creencia):
        self.creencias.add(creencia)  # Método para agregar una creencia al conjunto de creencias del agente

    def verificar_creencia(self, creencia):
        return creencia in self.creencias  # Método para verificar si una creencia está presente en el conjunto de creencias del agente

# Ejemplo de uso
if __name__ == "__main__":
    agente = AgenteModal()  # Creamos una instancia de la clase AgenteModal

    # El agente tiene algunas creencias sobre el mundo
    agente.agregar_creencia("Es necesario que llueva mañana.")  # Agregamos una creencia sobre la necesidad de lluvia
    agente.agregar_creencia("Es posible que hoy sea un día soleado.")  # Agregamos una creencia sobre la posibilidad de sol

    # Verificar algunas creencias
    print("El agente cree que es necesario que llueva mañana:", agente.verificar_creencia("Es necesario que llueva mañana."))  # Verificamos si el agente cree en la necesidad de lluvia
    print("El agente cree que es posible que nieve mañana:", agente.verificar_creencia("Es posible que nieve mañana."))  # Verificamos si el agente cree en la posibilidad de nieve
