#Victor Eduardo Aleman Padilla 21310193
# Definición de una clase para representar animales
class Animal:
    def __init__(self, nombre, tipo, habitat):
        self.nombre = nombre  # Nombre del animal
        self.tipo = tipo      # Tipo de animal (por ejemplo, mamífero, ave)
        self.habitat = habitat  # Hábitat del animal (por ejemplo, savannah, selva)

# Definición de algunos animales
leon = Animal("León", "Mamífero", "Savannah")  # Instancia de la clase Animal
tigre = Animal("Tigre", "Mamífero", "Selva")    # Instancia de la clase Animal
aguila = Animal("Águila", "Ave", "Montaña")     # Instancia de la clase Animal

# Ontología de animales
class OntologiaAnimales:
    def __init__(self):
        self.animales = []  # Lista para almacenar los animales

    def agregar_animal(self, animal):
        self.animales.append(animal)  # Método para agregar un animal a la ontología

    def encontrar_animal_por_tipo(self, tipo):
        return [animal for animal in self.animales if animal.tipo == tipo]  # Método para encontrar animales por tipo

    def encontrar_animal_por_habitat(self, habitat):
        return [animal for animal in self.animales if animal.habitat == habitat]  # Método para encontrar animales por hábitat

# Crear una instancia de la ontología de animales
ontologia_animales = OntologiaAnimales()

# Agregar los animales definidos a la ontología
ontologia_animales.agregar_animal(leon)
ontologia_animales.agregar_animal(tigre)
ontologia_animales.agregar_animal(aguila)

# Consultas a la ontología
print("Animales de tipo Mamífero:")
for animal in ontologia_animales.encontrar_animal_por_tipo("Mamífero"):
    print(animal.nombre)

print("\nAnimales que viven en la Selva:")
for animal in ontologia_animales.encontrar_animal_por_habitat("Selva"):
    print(animal.nombre)
