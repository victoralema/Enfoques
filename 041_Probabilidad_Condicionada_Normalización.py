#Victor Eduardo Aleman Padilla 21310193

def calcular_probabilidad_condicionada(evento_a, evento_b, matriz_probabilidad):
    """
    Calcula la probabilidad condicionada P(evento_a | evento_b) dada una matriz de probabilidades.

    Args:
        evento_a (str): Evento A (evento condicionado).
        evento_b (str): Evento B (evento condicionante).
        matriz_probabilidad (dict): Matriz de probabilidades representada como un diccionario de diccionarios.

    Returns:
        float: La probabilidad condicionada P(evento_a | evento_b).
    """
    # Verifica si el evento condicionante (evento_b) está presente en la matriz de probabilidades
    # y si el evento condicionado (evento_a) está asociado con el evento condicionante
    if evento_b in matriz_probabilidad and evento_a in matriz_probabilidad[evento_b]:
        # Retorna la probabilidad condicionada P(evento_a | evento_b) desde la matriz de probabilidades
        return matriz_probabilidad[evento_b][evento_a]
    else:
        # Si no se puede calcular la probabilidad condicionada, retorna 0.0
        return 0.0

def normalizar_probabilidades(probabilidades):
    """
    Normaliza una lista de probabilidades para que sumen 1.

    Args:
        probabilidades (dict): Un diccionario de probabilidades.

    Returns:
        dict: Diccionario de probabilidades normalizadas.
    """
    # Calcula la suma total de todas las probabilidades en el diccionario
    suma_total = sum(probabilidades.values())

    # Verifica si la suma total es mayor que 0 (para evitar división por cero)
    if suma_total > 0:
        # Crea un nuevo diccionario con probabilidades normalizadas
        probabilidades_normalizadas = {evento: prob / suma_total for evento, prob in probabilidades.items()}
    else:
        # Si la suma total es 0, retorna las probabilidades originales
        probabilidades_normalizadas = probabilidades

    # Retorna el diccionario de probabilidades normalizadas
    return probabilidades_normalizadas

# Ejemplo de uso
if __name__ == "__main__":
    # Definir una matriz de probabilidades (probabilidades condicionadas)
    matriz_probabilidad = {
        "Lluvioso": {"Nublado": 0.3, "Despejado": 0.2},
        "Nublado": {"Lluvioso": 0.4, "Despejado": 0.1},
        "Despejado": {"Lluvioso": 0.1, "Nublado": 0.2}
    }

    # Calcular la probabilidad condicionada P("Nublado" | "Lluvioso")
    evento_condicionado = "Nublado"
    evento_condicionante = "Lluvioso"
    prob_condicionada = calcular_probabilidad_condicionada(evento_condicionado, evento_condicionante, matriz_probabilidad)

    # Mostrar resultado de la probabilidad condicionada
    print(f"La probabilidad condicionada P('{evento_condicionado}' | '{evento_condicionante}') es: {prob_condicionada}")

    # Normalizar las probabilidades para el evento "Lluvioso"
    probabilidades_lluvioso = matriz_probabilidad["Lluvioso"]
    probabilidades_normalizadas = normalizar_probabilidades(probabilidades_lluvioso)

    # Mostrar resultado de las probabilidades normalizadas
    print("\nProbabilidades normalizadas para el evento 'Lluvioso':")
    for evento, prob in probabilidades_normalizadas.items():
        print(f"P('{evento}') = {prob:.2f}")
