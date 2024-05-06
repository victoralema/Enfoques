#Victor Eduardo Aleman Padilla 21310193


def calcular_probabilidad_a_priori(evento, posibles_resultados, probabilidades):
    """
    Calcula la probabilidad a priori de un evento dado un conjunto de posibles resultados
    y sus probabilidades iniciales.

    Args:
        evento (str): El evento del cual se quiere calcular la probabilidad a priori.
        posibles_resultados (list): Lista de posibles resultados.
        probabilidades (list): Lista de probabilidades correspondientes a los posibles resultados.

    Returns:
        float: La probabilidad a priori del evento especificado.
    """
    if evento in posibles_resultados:
        indice_evento = posibles_resultados.index(evento)
        return probabilidades[indice_evento]
    else:
        return 0.0  # Si el evento no está en la lista de posibles resultados, su probabilidad es 0

# Ejemplo de uso
if __name__ == "__main__":
    # Definir los posibles resultados y sus probabilidades iniciales
    posibles_resultados = ["Cara", "Sello"]
    probabilidades = [0.5, 0.5]  # Probabilidad 0.5 para cada resultado (moneda justa)

    # Calcular la probabilidad a priori del evento "Cara"
    evento_interes = "Cara"
    prob_a_priori = calcular_probabilidad_a_priori(evento_interes, posibles_resultados, probabilidades)

    # Mostrar resultado
    print(f"La probabilidad a priori del evento '{evento_interes}' es: {prob_a_priori}")
