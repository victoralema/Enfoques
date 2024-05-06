#Victor Eduardo Aleman Padilla 21310193

import numpy as np

class GraphGame:
    def __init__(self, graph, players, payoffs):
        self.graph = graph  # Grafo que representa las conexiones entre nodos
        self.players = players  # Lista de jugadores en el juego
        self.payoffs = payoffs  # Matriz de pagos de cada jugador para cada nodo
        
    def find_nash_equilibrium(self):
        num_nodes = len(self.graph)  # Número de nodos en el grafo
        nash_equilibria = []  # Lista para almacenar los equilibrios de Nash
        
        # Iterar sobre todos los nodos y verificar si son un equilibrio de Nash
        for node in range(num_nodes):
            is_equilibrium = True  # Suponemos que el nodo es un equilibrio hasta que se demuestre lo contrario
            
            # Para cada jugador, verificar si cambiar de estrategia aumenta su pago
            for player in self.players:
                current_payoff = self.payoffs[player][node]  # Pago actual del jugador en el nodo actual
                for neighbor in self.graph[node]:
                    neighbor_payoff = self.payoffs[player][neighbor]  # Pago del jugador en el nodo vecino
                    if neighbor_payoff > current_payoff:  # Si el pago en el vecino es mayor
                        is_equilibrium = False  # No es un equilibrio de Nash
                        break
                if not is_equilibrium:  # Si no es un equilibrio para este jugador, no lo es para ninguno
                    break
            
            if is_equilibrium:
                nash_equilibria.append(node)  # Agregar el nodo a la lista de equilibrios
        
        return nash_equilibria  # Devolver la lista de equilibrios de Nash
    
    def design_mechanism(self, objective_player):
        mechanism = {}  # Diccionario para almacenar el mecanismo diseñado
        
        # Para el jugador objetivo, diseñar un mecanismo que maximice su pago
        for player in self.players:
            if player == objective_player:
                mechanism[player] = self.design_strategy(objective_player)  # Diseñar la estrategia para el jugador objetivo
            else:
                mechanism[player] = "Observador"  # Otros jugadores solo observan, no participan
        
        return mechanism  # Devolver el mecanismo diseñado
    
    def design_strategy(self, player):
        num_nodes = len(self.graph)  # Número de nodos en el grafo
        best_strategy = None  # Mejor estrategia inicialmente es desconocida
        max_payoff = -np.inf  # Pago máximo inicialmente es negativo infinito
        
        # Encontrar la estrategia que maximiza el pago del jugador
        for node in range(num_nodes):
            if self.payoffs[player][node] > max_payoff:  # Si el pago en este nodo es mayor que el máximo actual
                max_payoff = self.payoffs[player][node]  # Actualizar el pago máximo
                best_strategy = node  # Actualizar la mejor estrategia
        
        return best_strategy  # Devolver la mejor estrategia encontrada

# Definición del grafo (representado como un diccionario de listas de adyacencia)
graph = {
    0: [1, 2],  # Nodo 0 está conectado a los nodos 1 y 2
    1: [0, 2],  # Nodo 1 está conectado a los nodos 0 y 2
    2: [0, 1]   # Nodo 2 está conectado a los nodos 0 y 1
}

# Lista de jugadores en el juego
players = ["Jugador A", "Jugador B"]

# Matriz de pagos de cada jugador para cada nodo del grafo
payoffs = {
    "Jugador A": [3, 2, 1],  # Pago del Jugador A en los nodos 0, 1 y 2
    "Jugador B": [2, 1, 3]   # Pago del Jugador B en los nodos 0, 1 y 2
}

# Creación del juego
game = GraphGame(graph, players, payoffs)

# Encontrar equilibrios de Nash en el juego
nash_equilibria = game.find_nash_equilibrium()
print("Equilibrios de Nash encontrados:", nash_equilibria)

# Diseñar un mecanismo para el Jugador A que maximice su pago
mechanism = game.design_mechanism("Jugador A")
print("Mecanismo diseñado para el Jugador A:", mechanism)
