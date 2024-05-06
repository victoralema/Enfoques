#Victor Eduardo Aleman Padilla 21310193
class Action:
    def __init__(self, name, preconditions, effects):
        self.name = name
        self.preconditions = preconditions
        self.effects = effects

    def is_possible(self, state):
        # Verifica si las precondiciones de la acción son satisfechas por el estado actual
        return all(precondition in state for precondition in self.preconditions)

class Graph:
    # Definición de la clase Graph para representar el grafo de planificación
    def __init__(self, actions, initial_state, goal_state):
        self.actions = actions  # Lista de acciones disponibles
        self.initial_state = initial_state  # Estado inicial
        self.goal_state = goal_state  # Estado objetivo

    def is_goal_state(self, state):
        # Comprueba si el estado dado coincide con el estado objetivo
        return all(prop in state for prop in self.goal_state)

    def apply_action(self, state, action):
        # Aplica una acción al estado actual y devuelve el nuevo estado resultante
        if all(precondition in state for precondition in action.preconditions):
            new_state = state.copy()
            for effect in action.effects:
                new_state.add(effect)
            return new_state
        else:
            return None

    def find_mutex(self, layer):
        # Encuentra mutexes (acciones mutuamente excluyentes) en una capa dada
        mutexes = set()
        for i in range(len(layer)):
            for j in range(i + 1, len(layer)):
                if self.are_mutex(layer[i], layer[j]):
                    mutexes.add((layer[i], layer[j]))
        return mutexes

    def are_mutex(self, a, b):
        # Comprueba si dos acciones son mutuamente excluyentes
        if a.negate() == b or b.negate() == a:
            return True
        return False

    def extract_plan(self, layers):
        # Extrae un plan a partir de las capas generadas por GRAPHPLAN
        plan = []
        goal_layer = layers[-1]
        current_state = set(self.initial_state)
        for goal in self.goal_state:
            if goal not in current_state:
                for action in goal_layer:
                    if action.is_possible(current_state):
                        plan.append(action)
                        current_state = self.apply_action(current_state, action)
                        break
        return plan

    def graphplan(self):
        # Implementación del algoritmo GRAPHPLAN
        layers = []
        layer = [Action("init", [], self.initial_state)]  # Capa inicial con el estado inicial
        layers.append(layer)

        while True:
            previous_layer = layers[-1]  # Última capa generada
            new_layer = []

            for action in self.actions:
                # Genera nuevas acciones que pueden ser aplicadas en esta capa
                if action not in previous_layer:
                    if action.is_possible(previous_layer):
                        new_layer.append(action)

            mutexes = self.find_mutex(new_layer)  # Encuentra mutexes en la nueva capa
            layers.append(new_layer)  # Añade la nueva capa al grafo de planificación

            if self.is_goal_state(layers[-1]):  # Comprueba si se alcanzó el estado objetivo
                return self.extract_plan(layers), layers  # Retorna el plan y las capas generadas

            if all(len(mutex_layer) == 0 for mutex_layer in mutexes):
                # Si no hay mutexes en ninguna capa, no se puede continuar
                return None, layers


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

    # Crear grafo y realizar planificación
    graph = Graph(actions, initial_state, goal_state)
    plan, layers = graph.graphplan()

    if plan:
        print("Plan encontrado:")
        for action in plan:
            print(action.name)
    else:
        print("No se pudo encontrar un plan.")
