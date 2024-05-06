#Victor Eduardo Aleman Padilla 21310193
class NonMonotonicLogicAgent:
    def __init__(self):
        self.beliefs = set()  # Conjunto para almacenar creencias

    def assert_belief(self, proposition):
        """
        Asserts a belief.
        """
        self.beliefs.add(proposition)  # Agrega la creencia al conjunto de creencias

    def retract_belief(self, proposition):
        """
        Retracts a belief.
        """
        if proposition in self.beliefs:  # Si la creencia está presente
            self.beliefs.remove(proposition)  # Retira la creencia del conjunto de creencias

    def check_belief(self, proposition):
        """
        Checks if a belief holds.
        """
        return proposition in self.beliefs  # Retorna True si la creencia está presente en el conjunto de creencias

    def query(self, proposition):
        """
        Queries if a proposition holds.
        """
        if proposition in self.beliefs:  # Si la creencia está presente
            return True  # Retorna True
        else:
            # Aquí se pueden agregar reglas de inferencia para manejar creencias no monótonas
            return False  # Retorna False


# Ejemplo de uso
if __name__ == "__main__":
    agent = NonMonotonicLogicAgent()

    # El agente asume algunas creencias
    agent.assert_belief("Puede llover mañana")  # Agrega la creencia de que puede llover mañana
    agent.assert_belief("Pedro llevará un paraguas si llueve")  # Agrega la creencia de que Pedro llevará un paraguas si llueve

    # Consultas
    print(agent.query("Pedro llevará un paraguas si llueve"))  # Consulta si Pedro llevará un paraguas si llueve
    print(agent.query("Pedro llevará un paraguas"))  # Consulta si Pedro llevará un paraguas (esto podría ser incierto)
    agent.retract_belief("Puede llover mañana")  # Retira una creencia
    print(agent.query("Pedro llevará un paraguas"))  # Consulta nuevamente si Pedro llevará un paraguas
