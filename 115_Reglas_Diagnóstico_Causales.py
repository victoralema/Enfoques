#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Regla que representa una regla de diagnóstico.
class Regla:
    def __init__(self, condiciones, conclusion):
        self.condiciones = condiciones  # Condiciones que deben cumplirse para que se aplique la regla.
        self.conclusion = conclusion    # La conclusión o diagnóstico asociado a la regla.

    # Método para evaluar si las condiciones de la regla están presentes en los síntomas.
    def evaluar(self, sintomas):
        # Verifica si todas las condiciones de la regla están presentes en los síntomas.
        if all(condicion in sintomas for condicion in self.condiciones):
            return self.conclusion  # Si todas las condiciones se cumplen, se devuelve la conclusión.
        else:
            return None  # Si no se cumplen todas las condiciones, se devuelve None.

# Definición de la clase MotorInferencia que realiza inferencias basadas en un conjunto de reglas.
class MotorInferencia:
    def __init__(self, reglas):
        self.reglas = reglas  # Lista de reglas de diagnóstico.

    # Método para inferir la causa probable de los síntomas.
    def inferir(self, sintomas):
        # Itera sobre cada regla en busca de una que se aplique a los síntomas.
        for regla in self.reglas:
            conclusion = regla.evaluar(sintomas)
            if conclusion:
                return conclusion  # Si se encuentra una regla aplicable, se devuelve su conclusión.
        return "No se pudo determinar la causa."  # Si ninguna regla se aplica, se devuelve este mensaje.

# Función principal del programa.
def main():
    # Definimos las reglas de diagnóstico con sus condiciones y conclusiones.
    regla_fiebre = Regla(["fiebre"], "Resfriado")
    regla_tos_y_dolor_garganta = Regla(["tos", "dolor de garganta"], "Gripe")
    regla_tos_y_dificultad_respirar = Regla(["tos", "dificultad para respirar"], "Bronquitis")
    regla_dolor_cabeza_y_fiebre = Regla(["dolor de cabeza", "fiebre"], "Migraña")

    # Creamos el motor de inferencia con las reglas definidas.
    motor = MotorInferencia([regla_fiebre, regla_tos_y_dolor_garganta, regla_tos_y_dificultad_respirar, regla_dolor_cabeza_y_fiebre])

    # Simulamos síntomas de un paciente.
    sintomas_paciente = ["fiebre", "tos"]

    # Utilizamos el motor de inferencia para diagnosticar.
    diagnostico = motor.inferir(sintomas_paciente)
    print("Los síntomas indican:", diagnostico)

# Llamamos a la función principal si este script es ejecutado directamente.
if __name__ == "__main__":
    main()
