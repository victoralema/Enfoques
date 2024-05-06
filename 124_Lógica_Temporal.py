#Victor Eduardo Aleman Padilla 21310193

class TemporalLogicAgent:
    def __init__(self):
        self.beliefs = {}  # Diccionario para almacenar creencias en diferentes momentos temporales

    def assert_belief(self, proposition, time):
        """
        Asserts a belief at a given time.
        """
        if time not in self.beliefs:  # Si el tiempo no está en el diccionario de creencias
            self.beliefs[time] = []  # Crea una nueva entrada en el diccionario
        self.beliefs[time].append(proposition)  # Agrega la creencia a la lista de creencias en ese tiempo

    def check_belief(self, proposition, time):
        """
        Checks if a belief holds at a given time.
        """
        if time in self.beliefs:  # Si el tiempo está en el diccionario de creencias
            return proposition in self.beliefs[time]  # Retorna True si la creencia está presente en ese tiempo
        return False  # Retorna False si no hay creencia en ese tiempo

    def query(self, proposition, time):
        """
        Queries if a proposition holds at a given time.
        """
        for t in range(time, -1, -1):  # Itera desde el tiempo actual hasta el tiempo 0
            if self.check_belief(proposition, t):  # Verifica si la creencia está presente en ese tiempo
                return True  # Retorna True si la creencia está presente en algún momento
        return False  # Retorna False si la creencia nunca se cumple


# Ejemplo de uso
if __name__ == "__main__":
    agent = TemporalLogicAgent()

    # El agente asume algunas creencias en diferentes momentos temporales
    agent.assert_belief("Llueve", 0)  # Llueve en el tiempo 0
    agent.assert_belief("Hace sol", 2)  # Hace sol en el tiempo 2
    agent.assert_belief("Hace frío", 3)  # Hace frío en el tiempo 3

    # Consultas
    print(agent.query("Llueve", 1))  # Consulta si llueve en el tiempo 1
    print(agent.query("Hace sol", 1))  # Consulta si hace sol en el tiempo 1
    print(agent.query("Hace frío", 3))  # Consulta si hace frío en el tiempo 3
