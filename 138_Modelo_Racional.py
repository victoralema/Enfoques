#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase ModeloProbabilistaRacional
class ModeloProbabilistaRacional:
    # Método de inicialización de la clase
    def __init__(self):
        # Probabilidades iniciales para lluvia y frío
        self.probabilidad_lluvia = 0.3
        self.probabilidad_frio = 0.4

    # Método para actualizar las probabilidades basadas en la evidencia
    def actualizar_probabilidades(self, evidencia):
        # Si la evidencia indica lluvia, se actualiza la probabilidad de lluvia
        if "llueve" in evidencia:
            self.probabilidad_lluvia = 0.8
        # Si la evidencia indica frío, se actualiza la probabilidad de frío
        if "hace frío" in evidencia:
            self.probabilidad_frio = 0.7

    # Método para tomar una decisión basada en las probabilidades actualizadas
    def tomar_decision(self):
        # Calcula la probabilidad conjunta de lluvia y frío
        probabilidad_conjunta = self.probabilidad_lluvia * self.probabilidad_frio
        # Si la probabilidad conjunta es mayor que 0.5, se decide llevar un paraguas y una chaqueta
        if probabilidad_conjunta > 0.5:
            return "Lleva un paraguas y una chaqueta"
        # Si la probabilidad conjunta es menor o igual a 0.5, no es necesario llevar un paraguas y una chaqueta
        else:
            return "No es necesario llevar un paraguas y una chaqueta"

# Ejemplo de uso del modelo
modelo = ModeloProbabilistaRacional()

# Observaciones o evidencia
evidencia = ["llueve", "hace frío"]

# Actualizar las probabilidades basadas en la evidencia
modelo.actualizar_probabilidades(evidencia)

# Tomar una decisión basada en las probabilidades actualizadas
decision = modelo.tomar_decision()
print(decision)
