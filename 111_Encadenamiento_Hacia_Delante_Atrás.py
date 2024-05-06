#Victor Eduardo Aleman Padilla 21310193
class SistemaExperto:
    def __init__(self, reglas):
        self.reglas = reglas  # Inicializa el sistema experto con un conjunto de reglas

    def encadenamiento_hacia_adelante(self, hechos):
        print("Utilizando encadenamiento hacia adelante...")
        for regla in self.reglas:  # Itera sobre cada regla del sistema experto
            antecedentes, consecuente = regla  # Extrae los antecedentes y el consecuente de la regla
            # Verifica si todos los antecedentes de la regla están presentes en los hechos y el consecuente no lo está
            if all(hecho in hechos for hecho in antecedentes) and consecuente not in hechos:
                print(f"Se cumple la regla {antecedentes} -> {consecuente}")  # Imprime la regla que se cumple
                hechos.add(consecuente)  # Agrega el consecuente a los hechos
        return hechos  # Devuelve los hechos actualizados

    def encadenamiento_hacia_atras(self, meta, hechos):
        print("Utilizando encadenamiento hacia atrás...")
        # Verifica si la meta ya está presente en los hechos
        if meta in hechos:
            return True

        for regla in self.reglas:  # Itera sobre cada regla del sistema experto
            antecedentes, consecuente = regla  # Extrae los antecedentes y el consecuente de la regla
            # Verifica si la meta coincide con el consecuente de alguna regla
            if meta == consecuente:
                print(f"Se está probando la regla {antecedentes} -> {consecuente}")
                metas_cumplidas = True
                # Verifica si todos los antecedentes de la regla están presentes en los hechos
                for antecedente in antecedentes:
                    if antecedente not in hechos:
                        # Realiza un encadenamiento hacia atrás para cada antecedente
                        metas_cumplidas = metas_cumplidas and self.encadenamiento_hacia_atras(antecedente, hechos)
                if metas_cumplidas:
                    hechos.add(consecuente)  # Agrega el consecuente a los hechos
                    return True
        return False


if __name__ == "__main__":
    # Define las reglas del sistema experto
    reglas = [
        (["edad >= 18", "es_estudiante"], "descuento"),
        (["edad >= 65"], "descuento"),
    ]

    # Crea una instancia del sistema experto con las reglas definidas
    sistema_experto = SistemaExperto(reglas)

    print("Bienvenido al sistema experto de descuentos en la tienda.")

    # Solicita al usuario que ingrese su edad y si es estudiante
    edad = int(input("Por favor, introduce tu edad: "))
    es_estudiante = input("¿Eres estudiante? (s/n): ").lower() == "s"

    # Establece los hechos iniciales basados en la entrada del usuario
    hechos = set(["edad >= " + str(edad), "es_estudiante" if es_estudiante else "no_es_estudiante"])
    
    print("\nDeterminando si eres elegible para un descuento...")
    
    # Encadenamiento hacia adelante
    hechos_por_adelante = sistema_experto.encadenamiento_hacia_adelante(hechos.copy())
    print("\nHechos obtenidos por encadenamiento hacia adelante:", hechos_por_adelante)

    # Encadenamiento hacia atrás
    meta = "descuento"
    if sistema_experto.encadenamiento_hacia_atras(meta, hechos.copy()):
        print("\n¡Felicidades! Eres elegible para un descuento.")
    else:
        print("\nLo siento, no eres elegible para un descuento.")

"""
Bienvenido al sistema experto de descuentos en la tienda.
Por favor, introduce tu edad: 25
¿Eres estudiante? (s/n): s

Determinando si eres elegible para un descuento...
Utilizando encadenamiento hacia adelante...
Se cumple la regla ['edad >= 18', 'es_estudiante'] -> descuento
Utilizando encadenamiento hacia atrás...
Se está probando la regla ['edad >= 18', 'es_estudiante'] -> descuento
¡Felicidades! Eres elegible para un descuento.
"""
