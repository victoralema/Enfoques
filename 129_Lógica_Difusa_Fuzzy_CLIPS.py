#Victor Eduardo Aleman Padilla 21310193
# Funciones de membresía para la temperatura
def membresia_temperatura_fria(x):
    if x <= 20:
        return 1
    elif x > 20 and x < 25:
        return (25 - x) / 5  # Función de membresía triangular
    else:
        return 0

def membresia_temperatura_normal(x):
    if x >= 15 and x <= 25:
        return (x - 15) / 10  # Función de membresía triangular
    elif x > 25 and x < 30:
        return (30 - x) / 5  # Función de membresía triangular
    else:
        return 0

def membresia_temperatura_caliente(x):
    if x >= 30:
        return 1
    elif x > 25 and x < 30:
        return (x - 25) / 5  # Función de membresía triangular
    else:
        return 0

# Funciones de membresía para la velocidad del ventilador
def membresia_velocidad_baja(x):
    if x <= 50:
        return 1
    elif x > 50 and x < 60:
        return (60 - x) / 10  # Función de membresía triangular
    else:
        return 0

def membresia_velocidad_media(x):
    if x >= 20 and x <= 50:
        return (x - 20) / 30  # Función de membresía triangular
    elif x > 50 and x < 80:
        return (80 - x) / 30  # Función de membresía triangular
    else:
        return 0

def membresia_velocidad_alta(x):
    if x >= 50:
        return 1
    elif x > 40 and x < 50:
        return (x - 40) / 10  # Función de membresía triangular
    else:
        return 0

# Reglas difusas
def calcular_velocidad(temperatura):
    # Calcula las contribuciones de cada regla
    velocidad_baja = min(membresia_temperatura_fria(temperatura), membresia_velocidad_baja(50))
    velocidad_media = min(membresia_temperatura_normal(temperatura), membresia_velocidad_media(50))
    velocidad_alta = min(membresia_temperatura_caliente(temperatura), membresia_velocidad_alta(50))

    # Calcula el numerador y el denominador de la velocidad resultante
    numerador = (velocidad_baja * 50) + (velocidad_media * 50) + (velocidad_alta * 100)
    denominador = velocidad_baja + velocidad_media + velocidad_alta

    # Evita la división por cero y devuelve la velocidad resultante
    if denominador == 0:
        return 0
    else:
        return numerador / denominador

# Simulación
temperatura_ambiente = 28  # Valor de ejemplo para la temperatura ambiente
velocidad_ventilador = calcular_velocidad(temperatura_ambiente)

# Imprime los resultados
print("Temperatura:", temperatura_ambiente)
print("Velocidad del ventilador:", velocidad_ventilador)
