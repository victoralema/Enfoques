#Victor Eduardo Aleman Padilla 21310193
class Interpreter:
    def __init__(self):
        # Constructor de la clase Interpreter, inicializa el diccionario de ubicaciones a objetos.
        self.location_objects = {
            "living room": "TV",
            "kitchen": "knife",
            "bedroom": "bed"
        }

    def interpret(self, sentence):
        # Método para interpretar una oración dada y devolver su significado.
        # Convertimos la oración a minúsculas para hacer coincidencias de forma insensible a mayúsculas.
        sentence = sentence.lower()
        
        # Inicializamos la variable para almacenar la interpretación.
        interpretation = None

        # Verificamos si alguna de las ubicaciones está presente en la oración.
        for location in self.location_objects:
            if location in sentence:
                # Si encontramos una ubicación, obtenemos el objeto asociado a esa ubicación.
                object_found = self.location_objects[location]
                interpretation = f"The {object_found} is in the {location}."
                break
        
        # Si no se encontró ninguna ubicación en la oración.
        if interpretation is None:
            interpretation = "I couldn't understand the sentence."

        return interpretation

def main():
    # Función principal del programa.
    interpreter = Interpreter()  # Creamos una instancia de la clase Interpreter.
    sentences = [
        "Where is the TV?",          # Lista de frases para interpretar.
        "I'm looking for the knife.",
        "Where's my bed?"
    ]

    for sentence in sentences:
        # Iteramos sobre cada frase en la lista y obtenemos su interpretación.
        interpretation = interpreter.interpret(sentence)
        print(interpretation)  # Imprimimos la interpretación de la frase.

if __name__ == "__main__":
    main()  # Llamamos a la función main cuando se ejecuta el script directamente.
