#Victor Eduardo Aleman Padilla 21310193
# Definir la clase Action (acción)
class Action:
    # Constructor de la clase Action
    def __init__(self, name, preconditions, effects):
        self.name = name  # Nombre de la acción
        self.preconditions = preconditions  # Precondiciones necesarias para que la acción pueda ejecutarse
        self.effects = effects  # Efectos de la acción sobre el estado del mundo

# Definir la clase State (estado)
class State:
    # Constructor de la clase State
    def __init__(self, predicates):
        self.predicates = predicates  # Predicados que describen el estado del mundo

    # Método para verificar si el estado satisface ciertas condiciones
    def satisfies(self, conditions):
        return all(condition in self.predicates for condition in conditions)

    # Método para aplicar una acción al estado y obtener el nuevo estado resultante
    def apply(self, action):
        if self.satisfies(action.preconditions):
            new_state = State(self.predicates.copy())  # Crear una copia del estado actual
            # Aplicar los efectos de la acción al nuevo estado
            new_state.predicates -= set(action.effects['delete'])
            new_state.predicates |= set(action.effects['add'])
            return new_state
        else:
            return None

# Definir la clase Planner (planificador)
class Planner:
    # Constructor de la clase Planner
    def __init__(self, initial_state, goal_state, actions):
        self.initial_state = initial_state  # Estado inicial del mundo
        self.goal_state = goal_state  # Estado objetivo que se desea alcanzar
        self.actions = actions  # Lista de acciones disponibles

    # Método para encontrar un plan que transforme el estado inicial en el estado objetivo
    def plan(self):
        plan = []  # Inicializar el plan como una lista vacía
        state = self.initial_state  # Establecer el estado actual como el estado inicial

        # Mientras el estado actual no satisfaga el estado objetivo
        while not state.satisfies(self.goal_state.predicates):
            # Encontrar las acciones aplicables al estado actual
            applicable_actions = [action for action in self.actions if state.satisfies(action.preconditions)]
            # Si no hay acciones aplicables, el planificador no puede encontrar un plan
            if not applicable_actions:
                print("No plan found!")
                return None

            # Seleccionar la primera acción aplicable y añadirla al plan
            action = applicable_actions[0]
            plan.append(action.name)
            # Aplicar la acción seleccionada al estado actual
            state = state.apply(action)

        return plan  # Devolver el plan encontrado

# Definir el estado inicial del mundo (predicados que describen el estado inicial)
initial_predicates = {'At(A, S)', 'At(B, D)', 'On(A, B)', 'Clear(A)', 'Clear(D)', 'Clear(C)', 'OnTable(B)', 'OnTable(C)'}

# Definir el estado objetivo que se desea alcanzar (predicados que describen el estado objetivo)
goal_predicates = {'At(A, D)', 'At(B, D)', 'At(C, D)'}

# Definir las acciones disponibles en el dominio
actions = [
    Action('Move(A, X, Y)', {'At(A, X)', 'Clear(A)', 'Clear(Y)'}, {'delete': {'At(A, X)', 'Clear(Y)'}, 'add': {'At(A, Y)', 'Clear(X)'}}),
    Action('MoveToTable(A, X)', {'At(A, X)', 'Clear(A)'}, {'delete': {'At(A, X)'}, 'add': {'At(A, S)', 'Clear(X)'}}),
    Action('MoveOver(A, X, Y)', {'At(A, X)', 'Clear(A)', 'On(A, X)', 'Clear(Y)'}, {'delete': {'At(A, X)', 'On(A, X)', 'Clear(Y)'}, 'add': {'At(A, Y)', 'Clear(X)'}})
]

# Crear instancias de los estados inicial y objetivo
initial_state = State(initial_predicates)
goal_state = State(goal_predicates)

# Crear un planificador y encontrar un plan
planner = Planner(initial_state, goal_state, actions)
plan = planner.plan()

# Imprimir el plan encontrado
if plan:
    print("Plan found:")
    for action in plan:
        print(action)
