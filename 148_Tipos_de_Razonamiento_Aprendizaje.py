#Victor Eduardo Aleman Padilla 21310193
class RazonamientoAI:
    def __init__(self):
        # Inicialización de la clase RazonamientoAI con una lista vacía para almacenar conocimiento
        self.conocimiento = []

    def agregar_conocimiento(self, conocimiento_nuevo):
        # Método para agregar nuevo conocimiento a la lista
        self.conocimiento.append(conocimiento_nuevo)

    def razonamiento_inductivo(self, evidencia):
        # Razonamiento inductivo: generalizar a partir de ejemplos específicos
        print("Razonamiento inductivo:")
        for item in evidencia:
            print("-", item)
        conclusion = "Los siguientes elementos también pueden ser parte del patrón."
        print("Conclusión:", conclusion)

    def razonamiento_deductivo(self, reglas):
        # Razonamiento deductivo: aplicar reglas para llegar a una conclusión específica
        print("\nRazonamiento deductivo:")
        for regla in reglas:
            print("- Si", regla[0], "entonces", regla[1])
        conclusion = "Dado que todas las condiciones se cumplen, la conclusión es:", reglas[-1][1]
        print("Conclusión:", conclusion)

    def actualizar_conocimiento(self, nueva_evidencia):
        # Simulación del aprendizaje: actualizar creencias con nueva evidencia
        print("\nActualizando conocimiento con nueva evidencia:")
        for item in nueva_evidencia:
            if item not in self.conocimiento:
                self.conocimiento.append(item)
        print("Nuevo conocimiento:", self.conocimiento)


# Ejemplo de uso del programa
if __name__ == "__main__":
    # Instanciación de un objeto RazonamientoAI
    AI = RazonamientoAI()

    # Datos de ejemplo
    ejemplos_inductivos = ["1, 2, 3, 4, 5", "rojo, verde, azul"]
    reglas_deductivas = [("x es un mamífero", "x tiene sangre caliente"),
                         ("x tiene sangre caliente", "x respira oxígeno"),
                         ("x es un perro", "x es un mamífero")]
    nueva_evidencia = ["rosa, amarillo", "x es un perro"]

    # Razonamiento inductivo
    AI.razonamiento_inductivo(ejemplos_inductivos)

    # Razonamiento deductivo
    AI.razonamiento_deductivo(reglas_deductivas)

    # Actualización del conocimiento con nueva evidencia
    AI.actualizar_conocimiento(nueva_evidencia)
