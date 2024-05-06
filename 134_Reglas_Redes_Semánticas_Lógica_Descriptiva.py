#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Regla que representa una regla en el sistema experto
class Regla:
    def __init__(self, antecedente, consecuente):
        self.antecedente = antecedente  # Conjunto de hechos que deben ser verdaderos para aplicar la regla
        self.consecuente = consecuente  # Conjunto de hechos que se deducen si el antecedente es verdadero

# Definición de la clase SistemaExperto que representa el sistema experto
class SistemaExperto:
    def __init__(self):
        self.reglas = []  # Lista para almacenar las reglas del sistema experto

    # Método para agregar una regla al sistema experto
    def agregar_regla(self, regla):
        self.reglas.append(regla)  # Agrega la regla a la lista de reglas

    # Método para hacer inferencia a partir de los hechos iniciales
    def hacer_inferencia(self, hechos):
        for regla in self.reglas:  # Itera sobre todas las reglas del sistema
            antecedente_verdadero = True  # Suponemos que el antecedente es verdadero inicialmente
            for hecho in regla.antecedente:  # Itera sobre cada hecho del antecedente de la regla
                if hecho not in hechos:  # Verifica si el hecho no está presente en los hechos iniciales
                    antecedente_verdadero = False  # Si falta un hecho del antecedente, la regla no se aplica
                    break
            if antecedente_verdadero:  # Si todos los hechos del antecedente están presentes
                for consecuente in regla.consecuente:  # Itera sobre cada hecho del consecuente de la regla
                    hechos.add(consecuente)  # Agrega el hecho del consecuente a los hechos iniciales

# Creamos una instancia de la clase SistemaExperto
sistema_experto = SistemaExperto()

# Creamos algunas reglas
regla1 = Regla({"Llueve"}, {"Humedad"})  # Si llueve, entonces hay humedad
regla2 = Regla({"Humedad"}, {"Niebla"})  # Si hay humedad, entonces hay niebla
regla3 = Regla({"Temperatura fría"}, {"Nieve"})  # Si hace frío, entonces hay nieve

# Agregamos las reglas al sistema experto
sistema_experto.agregar_regla(regla1)
sistema_experto.agregar_regla(regla2)
sistema_experto.agregar_regla(regla3)

# Definimos algunos hechos iniciales
hechos = {"Llueve", "Temperatura fría"}  # Definimos que llueve y hace frío inicialmente

# Hacemos inferencia a partir de los hechos iniciales
sistema_experto.hacer_inferencia(hechos)

# Mostramos los hechos resultantes después de la inferencia
print("Hechos resultantes después de la inferencia:", hechos)
