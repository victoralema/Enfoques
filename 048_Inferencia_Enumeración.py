#Victor Eduardo Aleman Padilla 21310193

# Importar las clases necesarias desde pgmpy
from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# Definir la estructura de la red bayesiana
model = BayesianNetwork([('D', 'G'), ('I', 'G'), ('G', 'L')])

# Definir las Tablas de Probabilidad Condicional (CPDs) para cada nodo
cpd_d = TabularCPD(variable='D', variable_card=2, values=[[0.6], [0.4]])
cpd_i = TabularCPD(variable='I', variable_card=2, values=[[0.7], [0.3]])
cpd_g = TabularCPD(variable='G', variable_card=3,
                   values=[[0.3, 0.05, 0.9, 0.5],
                           [0.4, 0.25, 0.08, 0.3],
                           [0.3, 0.7, 0.02, 0.2]],
                   evidence=['D', 'I'], evidence_card=[2, 2])
cpd_l = TabularCPD(variable='L', variable_card=2,
                   values=[[0.1, 0.4, 0.99],
                           [0.9, 0.6, 0.01]],
                   evidence=['G'], evidence_card=[3])

# Agregar las CPDs al modelo
model.add_cpds(cpd_d, cpd_i, cpd_g, cpd_l)

# Verificar la validez del modelo (comprobar si las CPDs son consistentes con la estructura)
model.check_model()

# Realizar inferencia por enumeración
inference = VariableElimination(model)

# Calcular la probabilidad P(L | D=0, I=1) mediante inferencia por enumeración
query_variable = 'L'
evidence = {'D': 0, 'I': 1}  # Definir la evidencia (D=0, I=1)
result = inference.query(variables=[query_variable], evidence=evidence)

# Imprimir los resultados de la consulta
print(f"Probabilidad de {query_variable} dado D=0 e I=1:")
print(result)

# Obtener los valores de la distribución de probabilidad de L dado D=0 e I=1 obtenida mediante inferencia por enumeración
prob_values = result.values[0]  # Obtener los valores de probabilidad como una lista
print(f"Valores de probabilidad: {prob_values}")
