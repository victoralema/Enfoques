#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Objeto que representará los objetos en nuestra taxonomía
class Objeto:
    def __init__(self, nombre):
        self.nombre = nombre
        self.categoria = None  # Inicializamos la categoría como nula

# Definición de la clase Categoria que representará las categorías en nuestra taxonomía
class Categoria:
    def __init__(self, nombre, padre=None):
        self.nombre = nombre
        self.padre = padre  # Guardamos una referencia al padre de la categoría (si tiene)
        self.hijos = []     # Lista para almacenar las subcategorías

    # Método para agregar una subcategoría a esta categoría
    def agregar_hijo(self, hijo):
        self.hijos.append(hijo)

# Definición de la clase Taxonomia que manejará toda la estructura de categorías y objetos
class Taxonomia:
    def __init__(self):
        self.categorias = []  # Lista para almacenar todas las categorías
        self.objetos = []      # Lista para almacenar todos los objetos

    # Método para agregar una categoría a la taxonomía
    def agregar_categoria(self, nombre, padre=None):
        categoria = Categoria(nombre, padre)
        if padre:             # Si tiene un padre, añadimos esta categoría como su hijo
            padre.agregar_hijo(categoria)
        self.categorias.append(categoria)  # Agregamos la categoría a la lista de categorías
        return categoria

    # Método para agregar un objeto a la taxonomía
    def agregar_objeto(self, nombre, categoria):
        objeto = Objeto(nombre)
        objeto.categoria = categoria  # Asignamos la categoría al objeto
        self.objetos.append(objeto)   # Agregamos el objeto a la lista de objetos

    # Método para encontrar una categoría por su nombre
    def encontrar_categoria(self, nombre):
        for categoria in self.categorias:
            if categoria.nombre == nombre:
                return categoria
        return None

    # Método para encontrar un objeto por su nombre
    def encontrar_objeto(self, nombre):
        for objeto in self.objetos:
            if objeto.nombre == nombre:
                return objeto
        return None

# Ejemplo de uso
if __name__ == "__main__":
    taxonomia = Taxonomia()

    # Crear categorías
    animalia = taxonomia.agregar_categoria("Animalia")
    mamiferos = taxonomia.agregar_categoria("Mamíferos", animalia)
    reptiles = taxonomia.agregar_categoria("Reptiles", animalia)

    # Crear objetos y asignar categorías
    taxonomia.agregar_objeto("Perro", mamiferos)
    taxonomia.agregar_objeto("Gato", mamiferos)
    taxonomia.agregar_objeto("Serpiente", reptiles)

    # Buscar objetos y categorías
    objeto = taxonomia.encontrar_objeto("Perro")
    if objeto:
        print(f"El objeto {objeto.nombre} pertenece a la categoría {objeto.categoria.nombre}")

    categoria = taxonomia.encontrar_categoria("Reptiles")
    if categoria:
        print(f"La categoría {categoria.nombre} tiene {len(categoria.hijos)} subcategorías")
