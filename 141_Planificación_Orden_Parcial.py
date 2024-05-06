#Victor Eduardo Aleman Padilla 21310193
class PartialOrderPlanner:
    def __init__(self):
        self.actions = []  # Lista de acciones
        self.ordering_constraints = []  # Restricciones de ordenamiento

    def add_action(self, action):
        # Añadir una acción a la lista de acciones
        self.actions.append(action)

    def add_ordering_constraint(self, action1, action2):
        # Añadir una restricción de ordenamiento entre dos acciones
        self.ordering_constraints.append((action1, action2))

    def is_valid_plan(self, plan):
        # Verificar si un plan dado satisface todas las restricciones de ordenamiento
        for action1, action2 in self.ordering_constraints:
            if action1 in plan and action2 in plan:
                # Comprueba si la acción2 está después de la acción1 en el plan
                if plan.index(action1) > plan.index(action2):
                    return False
        return True

    def generate_plans(self):
        # Generar todos los posibles planes que satisfacen las restricciones de ordenamiento
        plans = [[]]  # Comienza con una lista vacía de acciones
        for action in self.actions:
            new_plans = []
            for plan in plans:
                for i in range(len(plan) + 1):
                    new_plan = list(plan)
                    # Inserta la acción en todas las posiciones posibles del plan
                    new_plan.insert(i, action)
                    if self.is_valid_plan(new_plan):
                        new_plans.append(new_plan)
            plans = new_plans
        return plans

if __name__ == "__main__":
    # Crea una instancia del planificador de orden parcial
    planner = PartialOrderPlanner()

    # Definir las acciones del dominio
    planner.add_action("A")
    planner.add_action("B")
    planner.add_action("C")
    planner.add_action("D")

    # Definir las restricciones de ordenamiento
    planner.add_ordering_constraint("A", "B")
    planner.add_ordering_constraint("C", "D")

    # Generar y mostrar los planes posibles
    plans = planner.generate_plans()
    print("Planes generados:")
    for plan in plans:
        print(plan)
