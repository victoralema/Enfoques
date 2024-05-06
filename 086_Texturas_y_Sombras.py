#Victor Eduardo Aleman Padilla 21310193
import numpy as np
import matplotlib.pyplot as plt

# Función para generar una textura aleatoria
def generate_texture(size):
    texture = np.random.rand(size, size)
    return texture

# Función para generar sombras basadas en la textura
def generate_shadows(texture, shadow_prob):
    shadows = np.zeros_like(texture)
    for i in range(texture.shape[0]):
        for j in range(texture.shape[1]):
            if np.random.rand() < shadow_prob:
                shadows[i, j] = 1
    return shadows

# Parámetros de la textura y sombras
texture_size = 100
shadow_prob = 0.2

# Generar textura y sombras
texture = generate_texture(texture_size)
shadows = generate_shadows(texture, shadow_prob)

# Visualizar textura y sombras
plt.figure(figsize=(10, 5))

plt.subplot(1, 2, 1)
plt.title('Textura')
plt.imshow(texture, cmap='gray', interpolation='nearest')
plt.axis('off')

plt.subplot(1, 2, 2)
plt.title('Sombras')
plt.imshow(shadows, cmap='gray', interpolation='nearest')
plt.axis('off')

plt.show()
