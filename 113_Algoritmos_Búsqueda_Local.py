#Victor Eduardo Aleman Padilla 21310193
import random

class NQueensLocalSearch:
    def __init__(self, n):
        self.n = n  # Tamaño del tablero (N)
        self.board = []  # Tablero para almacenar las posiciones de las reinas

    def solve(self):
        # Inicializa el tablero con una disposición aleatoria de las reinas
        self.board = [random.randint(0, self.n - 1) for _ in range(self.n)]

        while True:
            # Calcula la puntuación actual del tablero
            current_score = self.calculate_score(self.board)
            if current_score == 0:
                # Si la puntuación es 0, se ha encontrado una solución
                return self.board

            # Encuentra la mejor vecindad y su puntuación
            best_neighbor, best_neighbor_score = self.find_best_neighbor(self.board)

            if best_neighbor_score >= current_score:
                # Si la mejor vecindad no mejora la puntuación actual, el algoritmo termina
                return self.board

            # Actualiza el tablero con la mejor vecindad encontrada
            self.board = best_neighbor

    def calculate_score(self, board):
        # Calcula la puntuación del tablero contando el número de reinas que se atacan entre sí
        score = 0
        for i in range(self.n):
            for j in range(i + 1, self.n):
                if board[i] == board[j] or abs(board[i] - board[j]) == j - i:
                    score += 1
        return score

    def find_best_neighbor(self, board):
        best_score = float('inf')  # Inicializa la mejor puntuación como infinito positivo
        best_neighbor = None  # Inicializa la mejor vecindad como nula

        for i in range(self.n):
            for j in range(self.n):
                if board[i] != j:
                    # Intenta mover la reina en la fila i a la columna j
                    new_board = board[:]  # Copia el tablero actual
                    new_board[i] = j  # Mueve la reina a la nueva posición
                    score = self.calculate_score(new_board)  # Calcula la puntuación de la nueva configuración
                    if score < best_score:
                        # Actualiza la mejor vecindad si se encuentra una con una puntuación mejor
                        best_score = score
                        best_neighbor = new_board

        return best_neighbor, best_score

def print_solution(solution):
    # Imprime el tablero con las reinas colocadas
    for row in solution:
        print(" ".join("Q" if cell == row else "-" for cell in range(len(solution))))
    print()

if __name__ == "__main__":
    n = int(input("Ingrese el tamaño del tablero (N): "))  # Solicita al usuario el tamaño del tablero
    n_queens = NQueensLocalSearch(n)  # Crea una instancia del problema de las N reinas
    solution = n_queens.solve()  # Resuelve el problema utilizando búsqueda local

    print("Solución encontrada:")  # Imprime un mensaje indicando la solución encontrada
    print_solution(solution)  # Imprime la solución encontrada en el tablero

#Ingrese el tamaño del tablero (N): 8
