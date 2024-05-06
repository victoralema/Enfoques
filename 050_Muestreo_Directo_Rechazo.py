#Victor Eduardo Aleman Padilla 21310193

import random
import math

def direct_sampling_normal_probability(lower_bound, upper_bound, num_samples):
    count_in_range = 0

    for _ in range(num_samples):
        sample = random.normalvariate(0, 1)  # Generar una muestra de la distribución normal estándar
        if lower_bound <= sample <= upper_bound:
            count_in_range += 1

    probability_estimate = count_in_range / num_samples
    return probability_estimate

# Estimación de la probabilidad de que Z ~ N(0, 1) caiga en [-1, 1]
lower_bound = -1
upper_bound = 1
num_samples = 10000

estimated_probability = direct_sampling_normal_probability(lower_bound, upper_bound, num_samples)
print("Estimación de la probabilidad (muestreo directo):", estimated_probability)
