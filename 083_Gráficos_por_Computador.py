#Victor Eduardo Aleman Padilla 21310193
import random
import matplotlib.pyplot as plt

# Función para estimar el valor de π utilizando el método de Monte Carlo
def estimar_pi(num_puntos):
    puntos_dentro_circulo = 0  # Contador para los puntos que caen dentro del círculo
    for _ in range(num_puntos):
        x = random.uniform(-1, 1)  # Genera una coordenada x aleatoria en el rango [-1, 1]
        y = random.uniform(-1, 1)  # Genera una coordenada y aleatoria en el rango [-1, 1]
        distancia_centro = x**2 + y**2  # Calcula la distancia al centro (0,0)
        if distancia_centro <= 1:  # Si la distancia es menor o igual a 1, el punto está dentro del círculo
            puntos_dentro_circulo += 1  # Incrementa el contador de puntos dentro del círculo
        # Dibuja el punto en azul si está dentro del círculo, rojo si está fuera
        plt.scatter(x, y, color='blue' if distancia_centro <= 1 else 'red', s=5)  
    # Estima el valor de π multiplicando por 4 la proporción de puntos dentro del círculo respecto al total
    pi_estimado = 4 * puntos_dentro_circulo / num_puntos  
    return pi_estimado

# Ejemplo de uso
num_puntos = 1000  # Número de puntos aleatorios a generar
pi_estimado = estimar_pi(num_puntos)  # Estima el valor de π
print(f"Valor estimado de π: {pi_estimado}")

# Ajusta el aspecto del gráfico para que los ejes tengan la misma escala
plt.gca().set_aspect('equal', adjustable='box')  
plt.title(f'Estimación de π con {num_puntos} puntos')  # Título del gráfico
plt.xlabel('X')  # Etiqueta del eje x
plt.ylabel('Y')  # Etiqueta del eje y
plt.show()  # Muestra el gráfico
