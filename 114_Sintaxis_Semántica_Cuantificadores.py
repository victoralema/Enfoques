#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Dominio que representa el conjunto de elementos sobre el cual se aplicarán los cuantificadores.
class Dominio:
    def __init__(self, elementos):
        self.elementos = elementos

    # Método para verificar si un elemento está contenido en el dominio.
    def contiene(self, elemento):
        return elemento in self.elementos

# Definición de la clase abstracta Cuantificador.
class Cuantificador:
    def __init__(self, dominio):
        self.dominio = dominio

    # Método abstracto para evaluar un predicado.
    def evaluar(self, predicado):
        raise NotImplementedError

# Definición de la clase CuantificadorUniversal que evalúa si un predicado es verdadero para todos los elementos en el dominio.
class CuantificadorUniversal(Cuantificador):
    def evaluar(self, predicado):
        # Itera sobre cada elemento del dominio.
        for elemento in self.dominio.elementos:
            # Si el predicado es falso para algún elemento, devuelve falso.
            if not predicado(elemento):
                return False
        # Si el predicado es verdadero para todos los elementos, devuelve verdadero.
        return True

# Definición de la clase CuantificadorExistencial que evalúa si un predicado es verdadero para al menos un elemento en el dominio.
class CuantificadorExistencial(Cuantificador):
    def evaluar(self, predicado):
        # Itera sobre cada elemento del dominio.
        for elemento in self.dominio.elementos:
            # Si el predicado es verdadero para algún elemento, devuelve verdadero.
            if predicado(elemento):
                return True
        # Si el predicado es falso para todos los elementos, devuelve falso.
        return False

# Función principal del programa.
def main():
    # Definimos el dominio como una lista de elementos.
    elementos = ['a', 'b', 'c', 'd']
    dominio = Dominio(elementos)

    # Definimos un predicado que verifica si una letra es una vocal.
    def es_vocal(letra):
        return letra in ['a', 'e', 'i', 'o', 'u']

    # Creamos un cuantificador universal y lo evaluamos con el predicado es_vocal.
    cuantificador_universal = CuantificadorUniversal(dominio)
    resultado_universal = cuantificador_universal.evaluar(es_vocal)
    print("¿Todas las letras son vocales?", resultado_universal)

    # Creamos un cuantificador existencial y lo evaluamos con el predicado es_vocal.
    cuantificador_existencial = CuantificadorExistencial(dominio)
    resultado_existencial = cuantificador_existencial.evaluar(es_vocal)
    print("¿Hay alguna letra que sea vocal?", resultado_existencial)

# Llamamos a la función principal si este script es ejecutado directamente.
if __name__ == "__main__":
    main()
