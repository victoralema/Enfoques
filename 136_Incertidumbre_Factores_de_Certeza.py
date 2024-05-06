#Victor Eduardo Aleman Padilla 21310193
# Funciones de membresía para temperatura
def membresia_frio(x):
    if x <= 30:  # Si la temperatura es <= 30, retorna 1 (totalmente frío)
        return 1
    elif x <= 50:  # Si la temperatura es <= 50, retorna un valor decreciente desde 1 hasta 0 conforme la temperatura aumenta
        return (50 - x) / 20
    else:  # Si la temperatura es > 50, retorna 0 (totalmente caliente)
        return 0

def membresia_calor(x):
    if x <= 50:  # Si la temperatura es <= 50, retorna 0 (totalmente frío)
        return 0
    elif x <= 70:  # Si la temperatura es <= 70, retorna un valor creciente desde 0 hasta 1 conforme la temperatura aumenta
        return (x - 50) / 20
    else:  # Si la temperatura es > 70, retorna 1 (totalmente caliente)
        return 1

# Funciones de membresía para sensación térmica
def membresia_frio_sensacion(x):
    if x <= 30:  # Si la sensación térmica es <= 30, retorna 1 (totalmente frío)
        return 1
    elif x <= 50:  # Si la sensación térmica es <= 50, retorna un valor decreciente desde 1 hasta 0 conforme la temperatura aumenta
        return (50 - x) / 20
    else:  # Si la sensación térmica es > 50, retorna 0 (totalmente caliente)
        return 0

def membresia_calor_sensacion(x):
    if x <= 50:  # Si la sensación térmica es <= 50, retorna 0 (totalmente frío)
        return 0
    elif x <= 70:  # Si la sensación térmica es <= 70, retorna un valor creciente desde 0 hasta 1 conforme la temperatura aumenta
        return (x - 50) / 20
    else:  # Si la sensación térmica es > 70, retorna 1 (totalmente caliente)
        return 1

# Inferencia
def inferencia(temperatura):
    if temperatura <= 30:  # Si la temperatura es <= 30, la inferencia es "frío"
        return "frío"
    elif temperatura >= 50:  # Si la temperatura es >= 50, la inferencia es "calor"
        return "calor"
    else:  # Si la temperatura está entre 30 y 50, la inferencia es "templado"
        return "templado"

# Definición de reglas
def reglas(temperatura):
    sensacion = inferencia(temperatura)  # Determina la inferencia de la temperatura
    if sensacion == "frío":  # Si la inferencia es "frío", utiliza la función de membresía correspondiente para la sensación de frío
        return membresia_frio_sensacion(temperatura)
    elif sensacion == "calor":  # Si la inferencia es "calor", utiliza la función de membresía correspondiente para la sensación de calor
        return membresia_calor_sensacion(temperatura)
    else:  # Si la inferencia es "templado", utiliza una membresía intermedia entre frío y calor
        return 1

# Calcular sensación térmica para una temperatura específica
temperatura = 40  # Define la temperatura
sensacion = reglas(temperatura)  # Calcula la sensación térmica utilizando las reglas definidas
print("Sensación térmica:", sensacion)  # Imprime la sensación térmica calculada
