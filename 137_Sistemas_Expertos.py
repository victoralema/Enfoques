#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase SistemaExperto
class SistemaExperto:
    # Método de inicialización de la clase
    def __init__(self):
        # Definición de la base de conocimiento como un diccionario
        self.base_conocimiento = {
            # Lista de reglas, cada regla es un diccionario con condiciones (si) y acciones (entonces)
            "reglas": [
                {"si": ["llueve"], "entonces": "lleva un paraguas"},  # Regla 1
                {"si": ["hace sol"], "entonces": "lleva gafas de sol"},  # Regla 2
                {"si": ["llueve", "hace frío"], "entonces": "lleva un paraguas y una chaqueta"},  # Regla 3
                {"si": ["hace sol", "hace calor"], "entonces": "lleva gafas de sol y protector solar"}  # Regla 4
            ]
        }

    # Método para inferir acciones basadas en las condiciones dadas
    def inferir(self, condiciones):
        # Lista para almacenar las acciones inferidas
        acciones = []
        # Iterar sobre cada regla en la base de conocimiento
        for regla in self.base_conocimiento["reglas"]:
            # Verificar si todas las condiciones de la regla están presentes en las condiciones dadas
            if all(condicion in condiciones for condicion in regla["si"]):
                # Si se cumplen todas las condiciones, agregar la acción correspondiente a la lista de acciones
                acciones.append(regla["entonces"])
        # Devolver la lista de acciones inferidas
        return acciones

# Ejemplo de uso del sistema experto
sistema = SistemaExperto()

# Condiciones del ambiente
condiciones_ambientales = ["llueve", "hace frío"]

# Inferir acciones basadas en las condiciones ambientales dadas
acciones_recomendadas = sistema.inferir(condiciones_ambientales)
# Imprimir las acciones recomendadas
print("Acciones recomendadas:", acciones_recomendadas)
