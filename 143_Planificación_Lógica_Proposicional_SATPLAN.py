#Victor Eduardo Aleman Padilla 21310193
class Action:
    def __init__(self, name, preconditions, effects):
        self.name = name  # Nombre de la acción
        self.preconditions = preconditions  # Precondiciones para que la acción sea válida
        self.effects = effects  # Efectos que tiene la acción sobre el estado

class SATPlan:
    def __init__(self, actions, initial_state, goal_state):
        self.actions = actions  # Lista de acciones disponibles
        self.initial_state = initial_state  # Estado inicial del problema de planificación
        self.goal_state = goal_state  # Estado objetivo que se desea alcanzar

    def encode(self):
        num_actions = len(self.actions)  # Número de acciones en el problema
        num_variables = len(self.initial_state) + num_actions  # Número total de variables en el problema SAT

        clauses = []  # Lista de cláusulas en la CNF (Forma Normal Conjuntiva)

        # Codificar que el estado inicial es verdadero
        for prop in self.initial_state:
            clauses.append([self.var_index(prop)])  # Añade una cláusula que establece que la variable correspondiente a la proposición del estado inicial es verdadera

        # Codificar que el estado objetivo es falso
        for prop in self.goal_state:
            clauses.append([-self.var_index(prop)])  # Añade una cláusula que establece que la negación de la variable correspondiente a la proposición del estado objetivo es verdadera

        # Codificar las acciones
        for action_index, action in enumerate(self.actions):
            preconditions = [self.var_index(prop) for prop in action.preconditions]  # Variables que representan las precondiciones de la acción
            effects = [self.var_index(prop) for prop in action.effects]  # Variables que representan los efectos de la acción

            # Si una acción se ejecuta, entonces sus precondiciones deben ser verdaderas y sus efectos también
            for effect in effects:
                clauses.append(preconditions + [effect])  # Añade una cláusula que establece que si la acción se ejecuta, sus precondiciones y efectos también son verdaderos
            # Al menos uno de los efectos de una acción debe ser verdadero
            clauses.append([-effect for effect in effects] + preconditions)  # Añade una cláusula que establece que al menos uno de los efectos debe ser verdadero si la acción se ejecuta

        return clauses, num_variables  # Devuelve las cláusulas y el número total de variables

    def var_index(self, prop):
        # Obtiene el índice de la variable para una proposición
        return abs(hash(prop)) + 1  # La función hash se utiliza para asignar un índice único a cada variable

    def decode_solution(self, solution):
        # Decodifica la solución SAT y devuelve el plan
        if solution is None:
            return None  # Si no hay solución, devuelve None

        plan = []  # Lista para almacenar el plan

        # Recorre la solución y agrega las acciones al plan si las variables correspondientes son verdaderas
        for index, value in enumerate(solution):
            if value > 0:
                if index <= len(self.initial_state):
                    continue
                else:
                    action_index = index - len(self.initial_state)
                    plan.append(self.actions[action_index])  # Añade la acción al plan si la variable es verdadera
        return plan

    def satplan(self):
        clauses, num_variables = self.encode()  # Codifica el problema de planificación como un problema SAT

        solution = self.solve_sat(clauses)  # Resuelve el problema SAT

        return self.decode_solution(solution)  # Decodifica la solución y devuelve el plan

    def solve_sat(self, clauses):
        # Resolver el problema SAT de manera muy simplificada
        variables = set()
        for clause in clauses:
            for literal in clause:
                variables.add(abs(literal))
        max_variable = max(variables)
        model = [1] * max_variable  # Se asume que todas las variables son verdaderas para esta implementación simplificada
        return model

if __name__ == "__main__":
    # Definir acciones
    actions = [
        Action("action1", ["A"], ["B"]),
        Action("action2", ["B"], ["C"]),
        Action("action3", ["C"], ["D"])
    ]

    # Definir estados iniciales y objetivos
    initial_state = ["A"]
    goal_state = ["D"]

    # Crear instancia de SATPlan y realizar la planificación
    sat_plan = SATPlan(actions, initial_state, goal_state)
    plan = sat_plan.satplan()

    # Imprimir el resultado del plan
    if plan:
        print("Plan encontrado:")
        for action in plan:
            print(action.name)  # Imprime el nombre de cada acción en el plan encontrado
    else:
        print("No se pudo encontrar un plan.")  # Imprime si no se pudo encontrar un plan
