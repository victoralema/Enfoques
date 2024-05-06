#Victor Eduardo Aleman Padilla 21310193
import random  # Importa el módulo random para generar números aleatorios

def monte_carlo_pi(num_dardos):
    dardos_dentro_circulo = 0  # Inicializa el contador de dardos dentro del círculo

    # Itera sobre el número de dardos especificado
    for _ in range(num_dardos):
        # Genera coordenadas aleatorias dentro del cuadrado unitario
        x = random.uniform(0, 1)
        y = random.uniform(0, 1)
        
        # Calcula la distancia al centro del cuadrado desde el punto generado
        distancia_al_centro = x**2 + y**2

        # Verifica si el punto generado está dentro del círculo unitario
        if distancia_al_centro <= 1:
            dardos_dentro_circulo += 1  # Incrementa el contador si está dentro del círculo

    # Estima el valor de pi usando la proporción de dardos dentro del círculo
    pi_estimado = 4 * (dardos_dentro_circulo / num_dardos)
    return pi_estimado  # Devuelve la estimación de pi

if __name__ == "__main__":
    # Solicita al usuario el número de dardos a lanzar
    num_dardos = int(input("Ingrese el número de dardos a lanzar: "))
    
    # Calcula la estimación de pi utilizando el método de Monte Carlo
    estimacion_pi = monte_carlo_pi(num_dardos)
    
    # Imprime la estimación de pi
    print(f"La estimación de pi utilizando {num_dardos} dardos es: {estimacion_pi}")
