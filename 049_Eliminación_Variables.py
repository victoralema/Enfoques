#Victor Eduardo Aleman Padilla 21310193

from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# Definir la estructura de la red Bayesiana
model = BayesianNetwork([('A', 'C'), ('B', 'C')])

# Definir las probabilidades condicionales
cpd_a = TabularCPD(variable='A', variable_card=2, values=[[0.5], [0.5]])
cpd_b = TabularCPD(variable='B', variable_card=2, values=[[0.3], [0.7]])
cpd_c = TabularCPD(variable='C', variable_card=2,
                   values=[[0.8, 0.9, 0.7, 0.1], [0.2, 0.1, 0.3, 0.9]],
                   evidence=['A', 'B'], evidence_card=[2, 2])

# Asociar las probabilidades condicionales al modelo
model.add_cpds(cpd_a, cpd_b, cpd_c)

# Verificar la validez de la red Bayesiana
model.check_model()

# Realizar inferencia por eliminación de variables
inference = VariableElimination(model)

# Consulta de inferencia P(C | A=0, B=1)
query = inference.query(variables=['C'], evidence={'A': 0, 'B': 1})
print(query)
