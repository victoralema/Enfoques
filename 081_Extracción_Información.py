#Victor Eduardo Aleman Padilla 21310193
import random

# Datos de ejemplo (puedes cambiarlos según tus necesidades)
datos = [
    {"nombre": "Juan", "edad": 25, "profesion": "ingeniero"},
    {"nombre": "María", "edad": 30, "profesion": "doctora"},
    {"nombre": "Carlos", "edad": 28, "profesion": "abogado"},
    {"nombre": "Laura", "edad": 35, "profesion": "ingeniera"},
    {"nombre": "Pedro", "edad": 40, "profesion": "médico"},
    {"nombre": "Ana", "edad": 27, "profesion": "abogada"}
]

# Función para calcular la probabilidad de que una persona tenga cierta profesión
def probabilidad_profesion(profesion, datos):
    total_personas = len(datos)  # Obtiene el total de personas en los datos
    personas_con_profesion = sum(1 for persona in datos if persona["profesion"] == profesion)  # Cuenta las personas con la profesión dada
    return personas_con_profesion / total_personas  # Calcula la probabilidad como el cociente entre las personas con la profesión dada y el total de personas

# Función para obtener una muestra aleatoria de datos basada en la probabilidad de una característica
def obtener_muestra_aleatoria(datos, caracteristica, valor, num_muestras):
    muestras = [persona for persona in datos if persona[caracteristica] == valor]  # Filtra las personas que tienen la característica y valor dados
    return random.sample(muestras, min(num_muestras, len(muestras)))  # Retorna una muestra aleatoria limitada por el número de muestras solicitadas y el tamaño de las muestras disponibles

# Ejemplo de uso
profesion = "ingeniero"
probabilidad = probabilidad_profesion(profesion, datos)  # Calcula la probabilidad de ser ingeniero
print(f"La probabilidad de que una persona sea {profesion} es: {probabilidad:.2f}")

muestra_aleatoria = obtener_muestra_aleatoria(datos, "profesion", profesion, 2)  # Obtiene una muestra aleatoria de personas que son ingenieras
print(f"Muestra aleatoria de {profesion}:")
for persona in muestra_aleatoria:
    print(persona)  # Imprime cada persona en la muestra aleatoria
