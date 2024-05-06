#Victor Eduardo Aleman Padilla 21310193


import numpy as np # Importa la librería NumPy para manipulación de matrices y cálculos numéricos eficientes.

class GraphQLearning: #Define una clase llamada GraphQLearning para encapsular la lógica del algoritmo Q-Learning.
    def __init__(self, num_nodes): #Constructor de la clase GraphQLearning. Inicializa los atributos de la clase, incluyendo la matriz Q, la tasa de aprendizaje (learning_rate) y el factor de descuento (discount_factor).
        self.num_nodes = num_nodes
        self.Q = np.zeros((num_nodes, num_nodes))  # Matriz Q inicializada con ceros
        self.learning_rate = 0.8 # Tasa de aprendizaje para actualizar los valores de Q
        self.discount_factor = 0.9 # Factor de descuento para el cálculo del retorno futuro

    def choose_action(self, state, possible_actions, epsilon): #Método para elegir una acción basada en una política epsilon-greedy (exploración vs explotación).
        if np.random.uniform(0, 1) < epsilon:
            # Exploración: elegir una acción aleatoria entre las posibles
            return np.random.choice(possible_actions)
        else:
            # Explotación: elegir la acción con el valor Q máximo para el estado actual
            Q_state = self.Q[state, possible_actions]
            return possible_actions[np.argmax(Q_state)]

    def update_q_value(self, state, next_state, action, reward): #Método para actualizar el valor Q del estado actual utilizando la ecuación de Q-Learning.
        best_next_action = np.argmax(self.Q[next_state])
        self.Q[state, action] += self.learning_rate * (reward + self.discount_factor * self.Q[next_state, best_next_action] - self.Q[state, action])

    def find_shortest_path(self, graph, start_node, goal_node, num_episodes=1000, epsilon=0.2): #Método para actualizar el valor Q del estado actual utilizando la ecuación de Q-Learning.
        for episode in range(num_episodes):
            current_state = start_node
            path = [current_state] # Inicializar la lista para almacenar el camino seguido
            
            while current_state != goal_node:
                possible_actions = np.where(graph[current_state] >= 0)[0]  # Obtener acciones válidas desde el estado actual
                action = self.choose_action(current_state, possible_actions, epsilon) # Elegir una acción
                next_state = action # El siguiente estado es la acción elegida
                reward = graph[current_state, next_state] # Obtener la recompensa por la transición
                
                # Actualizar el valor Q del estado actual
                self.update_q_value(current_state, next_state, action, reward)
                
                # Mover al siguiente estado
                current_state = next_state
                path.append(current_state) # Agregar el estado actual al camino seguido
            
            if path[-1] == goal_node:
                print(f"Camino más corto encontrado en el episodio {episode}: {path}")

        return self.Q # Devolver la matriz Q final después de completar los episodios de entrenamiento

# Ejemplo de uso
if __name__ == "__main__": #Bloque de código principal que se ejecuta cuando el script se ejecuta como un programa independiente.
    # Definir el grafo como una matriz de adyacencia ponderada
    # Aquí, -1 indica que no hay conexión entre los nodos
    graph = np.array([
        [-1, 2, -1, 1, -1, -1],  # Nodo 0
        [-1, -1, 3, -1, -1, -1], # Nodo 1
        [-1, -1, -1, -1, 4, -1],  # Nodo 2
        [-1, -1, -1, -1, 3, 7],   # Nodo 3
        [-1, -1, -1, -1, -1, 1],  # Nodo 4
        [-1, -1, -1, -1, -1, -1]  # Nodo 5 (Objetivo)
    ])

    num_nodes = len(graph)
    start_node = 0
    goal_node = 5

    ql = GraphQLearning(num_nodes) #Crea una instancia de la clase GraphQLearning con el número de nodos en el grafo.
    learned_Q = ql.find_shortest_path(graph, start_node, goal_node) #Ejecuta el algoritmo Q-Learning para encontrar el camino más corto desde start_node hasta goal_node en el grafo graph.

    print("\nMatriz Q final:") # Imprime un mensaje indicando que se va a mostrar la matriz Q final.
    print(learned_Q) #Imprime la matriz Q final aprendida después de ejecutar el algoritmo Q-Learning.
