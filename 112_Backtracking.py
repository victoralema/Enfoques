#Victor Eduardo Aleman Padilla 21310193
class NQueens:
    def __init__(self, n):
        self.n = n  # Tamaño del tablero (N)
        # Inicializa el tablero como una matriz de tamaño NxN con ceros
        self.board = [[0] * n for _ in range(n)]
        self.solutions = []  # Lista para almacenar las soluciones encontradas

    def solve(self):
        self._solve(0)  # Llama al método privado de resolución con la primera columna (col=0)
        return self.solutions  # Devuelve todas las soluciones encontradas

    def _solve(self, col):
        # Si todas las columnas han sido procesadas (col=N), se ha encontrado una solución
        if col == self.n:
            # Agrega una copia de la configuración actual del tablero a la lista de soluciones
            self.solutions.append([row[:] for row in self.board])
            return True

        # Itera sobre todas las filas en la columna actual
        for row in range(self.n):
            # Verifica si es seguro colocar una reina en la posición actual (row, col)
            if self.is_safe(row, col):
                # Coloca una reina en la posición actual
                self.board[row][col] = 1
                # Llama recursivamente al método _solve para la siguiente columna (col+1)
                self._solve(col + 1)
                # Deshace el cambio para probar otras posiciones (backtracking)
                self.board[row][col] = 0

    def is_safe(self, row, col):
        # Verifica si hay una reina en la misma fila en las columnas anteriores
        for i in range(col):
            if self.board[row][i] == 1:
                return False

        # Verifica si hay una reina en la diagonal superior izquierda
        for i, j in zip(range(row, -1, -1), range(col, -1, -1)):
            if self.board[i][j] == 1:
                return False

        # Verifica si hay una reina en la diagonal inferior izquierda
        for i, j in zip(range(row, self.n), range(col, -1, -1)):
            if self.board[i][j] == 1:
                return False

        return True

def print_solution(solution):
    # Imprime la solución representada por la matriz de tablero
    for row in solution:
        print(" ".join("Q" if cell == 1 else "-" for cell in row))
    print()

if __name__ == "__main__":
    # Solicita al usuario ingresar el tamaño del tablero (N)
    n = int(input("Ingrese el tamaño del tablero (N): "))
    # Crea una instancia de la clase NQueens con el tamaño del tablero dado
    n_queens = NQueens(n)
    # Encuentra todas las soluciones al problema de las N reinas
    solutions = n_queens.solve()

    # Imprime el número de soluciones encontradas
    print(f"Se encontraron {len(solutions)} soluciones para el problema de las {n} reinas:")
    # Imprime cada solución encontrada
    for i, solution in enumerate(solutions, 1):
        print(f"Solución {i}:")
        print_solution(solution)

        
#Ingrese el tamaño del tablero (N): 4
