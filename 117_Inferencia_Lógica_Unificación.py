#Victor Eduardo Aleman Padilla 21310193
class Unifier:
    def __init__(self):
        # Inicializa el objeto unificador con un diccionario vacío para almacenar las sustituciones.
        self.substitution = {}

    def unify(self, term1, term2):
        # Método para unificar dos términos.
        # Comprueba si los términos ya son iguales después de aplicar las sustituciones actuales.
        if self.substitute(term1) == self.substitute(term2):
            return self.substitution  # Si ya son iguales, devuelve las sustituciones actuales.
        # Si uno de los términos es una variable, asigna el otro término a esa variable.
        elif isinstance(term1, str) and term1.islower():
            self.substitution[term1] = term2
            return self.substitution
        elif isinstance(term2, str) and term2.islower():
            self.substitution[term2] = term1
            return self.substitution
        # Si ambos términos son listas, unifica cada elemento de las listas recursivamente.
        elif isinstance(term1, list) and isinstance(term2, list):
            if len(term1) != len(term2):
                return False  # Si las listas no tienen la misma longitud, la unificación falla.
            for t1, t2 in zip(term1, term2):
                self.unify(t1, t2)
            return self.substitution
        else:
            return False  # Si los términos no son iguales y no se puede realizar una unificación, devuelve False.

    def substitute(self, term):
        # Método para aplicar las sustituciones actuales a un término.
        if isinstance(term, str) and term in self.substitution:
            return self.substitution[term]  # Si el término es una variable sustituida, devuelve su valor.
        elif isinstance(term, list):
            # Si el término es una lista, aplica la sustitución a cada elemento de la lista.
            return [self.substitute(subterm) for subterm in term]
        else:
            return term  # Si el término no es una variable sustituida ni una lista, devuelve el término sin cambios.


def main():
    unifier = Unifier()  # Crea un objeto Unifier para realizar la unificación.

    # Ejemplo de unificación
    term1 = ['f', 'A', 'x']  # Primer término
    term2 = ['f', 'y', 'z']  # Segundo término
    substitution = unifier.unify(term1, term2)  # Intenta unificar los términos

    if substitution:
        print("Unificación exitosa:")
        for var, val in substitution.items():
            print(f"{var} = {val}")  # Imprime las sustituciones exitosas
    else:
        print("No se puede unificar.")  # Si no se puede realizar la unificación, imprime un mensaje.


if __name__ == "__main__":
    main()
