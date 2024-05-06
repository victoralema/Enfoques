#Victor Eduardo Aleman Padilla 21310193
class EightQueensProblem:
    def __init__(self, n=8):
        self.n = n  # Establece el tamaño del tablero (8 reinas por defecto)
        # Inicializa el tablero como una lista de listas, todas las casillas están inicializadas a 0 (no hay reinas)
        self.board = [[0 for _ in range(n)] for _ in range(n)]

    def is_safe(self, row, col):
        # Comprueba si es seguro colocar una reina en la posición (row, col)
        
        # Comprueba si hay alguna reina en la misma columna
        for i in range(row):
            if self.board[i][col] == 1:
                return False
        
        # Comprueba la diagonal superior izquierda
        for i, j in zip(range(row, -1, -1), range(col, -1, -1)):
            if self.board[i][j] == 1:
                return False
        
        # Comprueba la diagonal superior derecha
        for i, j in zip(range(row, -1, -1), range(col, self.n)):
            if self.board[i][j] == 1:
                return False
        
        # Si ninguna de las condiciones anteriores se cumple, es seguro colocar una reina en esta posición
        return True

    def solve_queens(self, row=0):
        # Función recursiva para resolver el problema de las reinas
        
        # Si todas las reinas se han colocado, se ha encontrado una solución
        if row == self.n:
            return True
        
        # Itera sobre cada columna en la fila actual
        for col in range(self.n):
            # Si es seguro colocar una reina en esta posición
            if self.is_safe(row, col):
                # Coloca una reina en esta posición
                self.board[row][col] = 1
                # Llama recursivamente a solve_queens para la siguiente fila
                if self.solve_queens(row + 1):
                    return True
                # Si no se encontró una solución, retrocede y prueba con otra columna
                self.board[row][col] = 0
        
        # Si no se encontró ninguna solución en esta fila, retrocede
        return False

    def print_solution(self):
        # Imprime la solución encontrada en el tablero
        for i in range(self.n):
            for j in range(self.n):
                print(self.board[i][j], end=" ")
            print()

if __name__ == "__main__":
    # Crea una instancia del problema de las ocho reinas
    problem = EightQueensProblem()
    # Intenta resolver el problema
    if problem.solve_queens():
        print("Solución encontrada:")
        # Si se encuentra una solución, imprímela
        problem.print_solution()
    else:
        # Si no se encuentra una solución, imprime un mensaje
        print("No se encontró solución.")
