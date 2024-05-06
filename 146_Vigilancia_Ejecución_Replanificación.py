#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Mundo para representar el estado del mundo y los objetivos
class Mundo:
    def __init__(self):
        self.estado_actual = {}  # Estado inicial del mundo
        self.objetivos = {}  # Objetivos a alcanzar

    def actualizar_estado(self, cambios):
        self.estado_actual.update(cambios)  # Método para actualizar el estado del mundo

    def estado_completo(self):
        # Método para obtener el estado completo del mundo
        return self.estado_actual


# Definición de la clase Planificador para generar y replanificar planes
class Planificador:
    def __init__(self, mundo):
        self.mundo = mundo  # Se asigna el mundo al planificador

    def plan_inicial(self):
        # Genera un plan inicial para alcanzar los objetivos dados el estado actual del mundo
        plan = {}  # Aquí se generaría el plan inicial
        return plan

    def replanificar(self):
        # Método para replanificar en caso de desviaciones significativas
        nuevo_plan = {}  # Aquí se generaría un nuevo plan
        return nuevo_plan


# Definición de la clase SistemaVigilancia para monitorear la ejecución y replanificar si es necesario
class SistemaVigilancia:
    def __init__(self, mundo, planificador):
        self.mundo = mundo
        self.planificador = planificador

    def vigilar_ejecucion(self):
        # Implementa un sistema de monitoreo continuo para vigilar la ejecución del plan
        while not objetivos_alcanzados(self.mundo.estado_completo(), self.mundo.objetivos):
            estado_actual = self.mundo.estado_completo()
            plan_actual = self.planificador.plan_inicial()

            print("Estado actual del mundo:", estado_actual)
            print("Plan actual:", plan_actual)

            if not plan_en_ejecucion():
                print("Iniciando ejecución del plan...")

            if desviaciones_detectadas():
                print("Desviaciones detectadas. Replanificando...")

            nuevo_plan = self.planificador.replanificar()
            print("Nuevo plan:", nuevo_plan)

# Función para verificar si se han alcanzado todos los objetivos del plan
def objetivos_alcanzados(estado_actual, objetivos):
    for objetivo, valor in objetivos.items():
        if objetivo not in estado_actual or estado_actual[objetivo] != valor:
            return False
    return True


# Función que devuelve verdadero si hay un plan en ejecución
def plan_en_ejecucion():
    print("Verificando si hay un plan en ejecución...")
    # Lógica para verificar si hay un plan en ejecución
    return False

def desviaciones_detectadas():
    print("Detectando desviaciones...")
    # Lógica para detectar desviaciones
    return False

def ejecutar_plan(plan):
    print("Ejecutando el plan...")
    # Lógica para ejecutar el plan
    pass


# Ejemplo de uso del sistema de vigilancia
mundo = Mundo()
mundo.actualizar_estado({"posicion_robot": (0, 0), "objetivo": "limpiar_sala"})
mundo.objetivos = {"objetivo": "limpiar_sala"}

planificador = Planificador(mundo)
sistema_vigilancia = SistemaVigilancia(mundo, planificador)
sistema_vigilancia.vigilar_ejecucion()
