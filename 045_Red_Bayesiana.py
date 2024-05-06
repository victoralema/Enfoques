#Victor Eduardo Aleman Padilla 21310193

# Importar la clase BayesianNetwork en lugar de BayesianModel
from pgmpy.models import BayesianNetwork

from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# Definir la estructura de la red bayesiana
model = BayesianNetwork([('S', 'D'), ('T', 'D')])

# Definir las probabilidades condicionales
cpd_s = TabularCPD(variable='S', variable_card=2, values=[[0.5], [0.5]])
cpd_t = TabularCPD(variable='T', variable_card=2, values=[[0.7], [0.3]])
cpd_d = TabularCPD(variable='D', variable_card=2,
                   values=[[0.95, 0.6, 0.9, 0.3],
                           [0.05, 0.4, 0.1, 0.7]],
                   evidence=['S', 'T'], evidence_card=[2, 2])

# Asociar las probabilidades condicionales al modelo
model.add_cpds(cpd_s, cpd_t, cpd_d)

# Verificar consistencia de la red bayesiana
print("Red Bayesiana válida?", model.check_model())

# Realizar inferencia utilizando Variable Elimination
inference = VariableElimination(model)

# Calcular la probabilidad de enfermedad (Dada Síntomas=Sí, Término=Tí)
query = inference.query(variables=['D'], evidence={'S': 1, 'T': 1})
print(query)
