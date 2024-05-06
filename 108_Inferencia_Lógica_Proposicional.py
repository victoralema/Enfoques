#Victor Eduardo Aleman Padilla 21310193
class Clausula:
    def __init__(self, literales):
        self.literales = set(literales)  # Inicializa la cláusula con un conjunto de literales

    def __repr__(self):
        return " | ".join(self.literales)  # Devuelve una representación de cadena de la cláusula

class InferenciaLogica:
    def __init__(self):
        self.clausulas = []  # Inicializa la base de conocimiento como una lista de cláusulas

    def agregar_clausula(self, clausula):
        self.clausulas.append(clausula)  # Agrega una cláusula a la base de conocimiento

    def resolver(self, clausula_1, clausula_2):
        for literal in clausula_1.literales:  # Para cada literal en la primera cláusula
            if literal.startswith("~"):  # Si el literal comienza con "~" (negación)
                complemento = literal[1:]  # Obtiene el literal sin la negación
            else:
                complemento = "~" + literal  # Agrega "~" al comienzo para formar el complemento
            if complemento in clausula_2.literales:  # Si el complemento está en la segunda cláusula
                nueva_clausula = Clausula((clausula_1.literales | clausula_2.literales) - {literal, complemento})
                # Crea una nueva cláusula combinando las cláusulas anteriores y elimina los literales cancelados
                if len(nueva_clausula.literales) == 0:
                    return True  # Se ha encontrado una contradicción
                self.agregar_clausula(nueva_clausula)  # Agrega la nueva cláusula a la base de conocimiento
        return False  # No se encontró ninguna contradicción

    def inferir(self):
        while True:
            nuevas_clausulas = []  # Lista para almacenar cláusulas derivadas
            for i in range(len(self.clausulas)):
                for j in range(i + 1, len(self.clausulas)):
                    if self.resolver(self.clausulas[i], self.clausulas[j]):
                        return True  # Se ha encontrado una contradicción
            if nuevas_clausulas == []:
                return False  # No se puede inferir más, se detiene el bucle

# Ejemplo de uso
if __name__ == "__main__":
    inferencia = InferenciaLogica()  # Se crea una instancia del objeto InferenciaLogica

    # Agregamos cláusulas (representando proposiciones) a la base de conocimiento
    inferencia.agregar_clausula(Clausula(["A", "~B", "C"]))
    inferencia.agregar_clausula(Clausula(["~A", "B"]))
    inferencia.agregar_clausula(Clausula(["~C", "A"]))

    # Realizamos la inferencia
    resultado = inferencia.inferir()  # Llama al método inferir para realizar la inferencia lógica

    if resultado:
        print("Se ha encontrado una contradicción. Por lo tanto, la inferencia es verdadera.")
    else:
        print("No se ha encontrado una contradicción. Por lo tanto, la inferencia es falsa.")
