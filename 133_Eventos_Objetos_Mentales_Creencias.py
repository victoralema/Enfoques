#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Creencia que representa una creencia individual del agente
class Creencia:
    def __init__(self, nombre, valor):
        self.nombre = nombre  # Nombre de la creencia
        self.valor = valor    # Valor de la creencia (puede ser True, False, u otro valor)

# Definición de la clase Agente que representa al agente con su sistema de creencias
class Agente:
    def __init__(self):
        self.creencias = {}  # Diccionario para almacenar las creencias del agente

    # Método para agregar una creencia al agente
    def agregar_creencia(self, creencia):
        self.creencias[creencia.nombre] = creencia.valor  # Agrega la creencia al diccionario de creencias

    # Método para actualizar una creencia existente en el agente
    def actualizar_creencia(self, nombre, nuevo_valor):
        if nombre in self.creencias:  # Verifica si la creencia existe en el diccionario
            self.creencias[nombre] = nuevo_valor  # Actualiza el valor de la creencia
        else:
            print("La creencia {} no existe en el agente.".format(nombre))  # Mensaje de error si la creencia no existe

    # Método para mostrar todas las creencias del agente
    def ver_creencias(self):
        if self.creencias:  # Verifica si el diccionario de creencias no está vacío
            print("Creencias del agente:")
            for nombre, valor in self.creencias.items():  # Itera sobre las creencias y sus valores
                print("- {}: {}".format(nombre, valor))  # Muestra el nombre de la creencia y su valor
        else:
            print("El agente no tiene creencias.")  # Mensaje si el agente no tiene creencias

# Creamos una instancia de la clase Agente
agente = Agente()

# Creamos algunas creencias como instancias de la clase Creencia
creencia1 = Creencia("Llueve", True)
creencia2 = Creencia("Tiene_hambre", False)
creencia3 = Creencia("Dia_libre", True)

# Agregamos las creencias al agente utilizando el método agregar_creencia
agente.agregar_creencia(creencia1)
agente.agregar_creencia(creencia2)
agente.agregar_creencia(creencia3)

# Mostramos las creencias del agente utilizando el método ver_creencias
agente.ver_creencias()

# Actualizamos una creencia existente utilizando el método actualizar_creencia
agente.actualizar_creencia("Tiene_hambre", True)

# Mostramos las creencias actualizadas del agente
agente.ver_creencias()
