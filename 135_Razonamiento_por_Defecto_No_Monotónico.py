#Victor Eduardo Aleman Padilla 21310193
class Regla:
    def __init__(self, antecedente, consecuente):
        self.antecedente = antecedente  # Guarda el antecedente de la regla
        self.consecuente = consecuente  # Guarda el consecuente de la regla

class MotorInferencia:
    def __init__(self):
        self.hechos = set()  # Conjunto para almacenar los hechos
        self.reglas = []      # Lista para almacenar las reglas

    def agregar_hecho(self, hecho):
        self.hechos.add(hecho)  # Agrega un hecho al conjunto de hechos

    def agregar_regla(self, regla):
        self.reglas.append(regla)  # Agrega una regla a la lista de reglas

    def aplicar_reglas(self):
        nuevos_hechos = True  # Bandera para indicar si se han agregado nuevos hechos
        while nuevos_hechos:
            nuevos_hechos = False  # Se restablece la bandera antes de comenzar a aplicar las reglas
            for regla in self.reglas:  # Itera sobre todas las reglas
                # Verifica si todos los antecedentes de la regla están presentes en los hechos
                if all(antecedente in self.hechos for antecedente in regla.antecedente):
                    for consecuente in regla.consecuente:  # Itera sobre los consecuentes de la regla
                        if consecuente not in self.hechos:  # Verifica si el consecuente no está presente en los hechos
                            self.hechos.add(consecuente)   # Agrega el consecuente a los hechos
                            nuevos_hechos = True   # Establece la bandera como verdadera ya que se agregó un nuevo hecho

# Definición de reglas
regla1 = Regla(['p'], ['q'])  # Regla: Si p entonces q
regla2 = Regla(['q'], ['r'])  # Regla: Si q entonces r

# Inicialización del motor de inferencia
motor = MotorInferencia()

# Agregar reglas al motor
motor.agregar_regla(regla1)
motor.agregar_regla(regla2)

# Agregar hecho inicial
motor.agregar_hecho('p')

# Aplicar reglas hasta que no haya nuevos hechos
motor.aplicar_reglas()

# Imprimir hechos resultantes
print("Hechos resultantes:", motor.hechos)
