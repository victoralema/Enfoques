#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Evento que representa un evento con nombre, precondiciones y efectos
class Evento:
    def __init__(self, nombre, precondiciones=None, efectos=None):
        self.nombre = nombre  # Nombre del evento
        self.precondiciones = precondiciones if precondiciones else []  # Lista de precondiciones del evento
        self.efectos = efectos if efectos else []  # Lista de efectos del evento

# Definición de la clase Situacion que representa una situación con nombre y una lista de eventos
class Situacion:
    def __init__(self, nombre, eventos=None):
        self.nombre = nombre  # Nombre de la situación
        self.eventos = eventos if eventos else []  # Lista de eventos que ocurren en esta situación

# Definición de la clase Accion que representa una acción con nombre, precondiciones y efectos
class Accion:
    def __init__(self, nombre, precondiciones=None, efectos=None):
        self.nombre = nombre  # Nombre de la acción
        self.precondiciones = precondiciones if precondiciones else []  # Lista de precondiciones de la acción
        self.efectos = efectos if efectos else []  # Lista de efectos de la acción

# Definición de la clase Marco que representa el marco de acciones, situaciones y eventos
class Marco:
    def __init__(self, situacion_inicial, acciones, situaciones, eventos):
        self.situacion_inicial = situacion_inicial  # Situación inicial del marco
        self.acciones = acciones  # Lista de acciones disponibles en el marco
        self.situaciones = situaciones  # Lista de situaciones en el marco
        self.eventos = eventos  # Lista de eventos en el marco

    # Método para ejecutar una acción dada y aplicar sus efectos
    def ejecutar_accion(self, accion):
        if accion in self.acciones:  # Verificar si la acción está en la lista de acciones disponibles
            print(f"Ejecutando acción: {accion.nombre}")
            # Aplicar efectos de la acción
            for efecto in accion.efectos:
                print(f"Aplicando efecto: {efecto.nombre}")
            print("Acción completada.")
        else:
            print("Acción no reconocida.")

# Ejemplo de uso
if __name__ == "__main__":
    # Definir eventos
    evento1 = Evento("Evento1")
    evento2 = Evento("Evento2")

    # Definir situaciones
    situacion_inicial = Situacion("SituacionInicial", [evento1])
    situacion_final = Situacion("SituacionFinal", [evento2])

    # Definir acciones
    accion1 = Accion("Accion1", efectos=[evento2])

    # Crear marco
    marco = Marco(situacion_inicial, [accion1], [situacion_inicial, situacion_final], [evento1, evento2])

    # Ejecutar acción
    marco.ejecutar_accion(accion1)
