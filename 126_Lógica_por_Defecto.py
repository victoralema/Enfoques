#Victor Eduardo Aleman Padilla 21310193
class DefaultLogicAgent:
    def __init__(self):
        self.default_beliefs = set()  # Conjunto para almacenar creencias por defecto
        self.strict_beliefs = set()   # Conjunto para almacenar creencias estrictas

    def add_default_belief(self, proposition):
        """
        Adds a default belief.
        """
        self.default_beliefs.add(proposition)  # Agrega la creencia por defecto al conjunto

    def add_strict_belief(self, proposition):
        """
        Adds a strict belief.
        """
        self.strict_beliefs.add(proposition)   # Agrega la creencia estricta al conjunto

    def check_belief(self, proposition):
        """
        Checks if a belief holds.
        """
        if proposition in self.strict_beliefs:  # Si la creencia es estricta
            return True  # Retorna True directamente
        elif proposition in self.default_beliefs:  # Si la creencia es por defecto
            # Aquí podríamos implementar un razonamiento para verificar si la creencia es válida
            return True  # Por defecto, se considera válida
        else:
            return False  # Retorna False si la creencia no está presente

    def retract_belief(self, proposition):
        """
        Retracts a belief.
        """
        if proposition in self.default_beliefs:
            self.default_beliefs.remove(proposition)  # Retira la creencia por defecto

    def retract_strict_belief(self, proposition):
        """
        Retracts a strict belief.
        """
        if proposition in self.strict_beliefs:
            self.strict_beliefs.remove(proposition)  # Retira la creencia estricta


# Ejemplo de uso
if __name__ == "__main__":
    agent = DefaultLogicAgent()

    # El agente asume algunas creencias por defecto
    agent.add_default_belief("Todos los pájaros vuelan")

    # El agente asume algunas creencias estrictas
    agent.add_strict_belief("Este pájaro es un pingüino")

    # Consultas
    print(agent.check_belief("Todos los pájaros vuelan"))  # Consulta sobre la creencia por defecto
    print(agent.check_belief("Este pájaro es un pingüino"))  # Consulta sobre la creencia estricta

    # Retractando creencias
    agent.retract_belief("Todos los pájaros vuelan")  # Retracta una creencia por defecto
    agent.retract_strict_belief("Este pájaro es un pingüino")  # Retracta una creencia estricta

    # Consultas después de retractar
    print(agent.check_belief("Todos los pájaros vuelan"))  # Consulta nuevamente la creencia por defecto
    print(agent.check_belief("Este pájaro es un pingüino"))  # Consulta nuevamente la creencia estricta
