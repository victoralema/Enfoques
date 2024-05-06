#Victor Eduardo Aleman Padilla 21310193
# Definición de la clase Pelicula para representar películas con diferentes atributos.
class Pelicula:
    def __init__(self, titulo, genero, duracion, calificacion):
        self.titulo = titulo  # Título de la película.
        self.genero = genero  # Género de la película.
        self.duracion = duracion  # Duración de la película en minutos.
        self.calificacion = calificacion  # Calificación de la película.

# Base de conocimiento: lista de películas con sus atributos.
base_conocimiento = [
    Pelicula("Titanic", "Romance", 195, 7.8),
    Pelicula("Inception", "Ciencia Ficción", 148, 8.8),
    Pelicula("The Shawshank Redemption", "Drama", 142, 9.3),
    Pelicula("The Dark Knight", "Acción", 152, 9.0),
    Pelicula("Interstellar", "Ciencia Ficción", 169, 8.6),
    Pelicula("Forrest Gump", "Drama", 142, 8.8),
    Pelicula("The Matrix", "Acción", 136, 8.7),
]

# Función para recomendar películas basadas en las preferencias del usuario.
def recomendar_pelicula(genero, duracion_minima, calificacion_minima):
    peliculas_recomendadas = []

    # Filtrar películas que cumplan con los criterios especificados.
    for pelicula in base_conocimiento:
        if pelicula.genero == genero and pelicula.duracion >= duracion_minima and pelicula.calificacion >= calificacion_minima:
            peliculas_recomendadas.append(pelicula.titulo)

    return peliculas_recomendadas

# Función principal del programa.
def main():
    # Datos de entrada del usuario.
    genero_usuario = input("¿Qué género de película prefieres? ")  # Género de la película deseada.
    duracion_minima_usuario = int(input("¿Cuál es la duración mínima que deseas (en minutos)? "))  # Duración mínima deseada.
    calificacion_minima_usuario = float(input("¿Cuál es la calificación mínima que deseas? "))  # Calificación mínima deseada.

    # Obtener recomendaciones de películas basadas en las preferencias del usuario.
    recomendaciones = recomendar_pelicula(genero_usuario, duracion_minima_usuario, calificacion_minima_usuario)

    # Mostrar las recomendaciones al usuario.
    if recomendaciones:
        print("Te recomendamos las siguientes películas:")
        for pelicula in recomendaciones:
            print("-", pelicula)
    else:
        print("Lo siento, no encontramos películas que coincidan con tus preferencias.")

# Llamamos a la función principal si este script es ejecutado directamente.
if __name__ == "__main__":
    main()
