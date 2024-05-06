#Victor Eduardo Aleman Padilla 21310193

class AgenteLogico:
    def __init__(self):
        # Definir conocimientos iniciales del agente
        self.hechos = set()  # Conjunto de hechos observados

    def observar(self, hecho):
        # El agente observa un nuevo hecho
        self.hechos.add(hecho)

    def decidir_accion(self):
        # Lógica de decisión basada en los hechos observados

        # Ejemplo de reglas lógicas simples
        if 'llueve' in self.hechos:  # Si el hecho "llueve" está en los hechos observados
            return "Llevar paraguas"  # El agente decide llevar un paraguas
        elif 'soleado' in self.hechos:  # Si el hecho "soleado" está en los hechos observados
            return "Llevar gafas de sol"  # El agente decide llevar gafas de sol
        else:
            return "No estoy seguro qué llevar"  # Si no se cumplen las condiciones anteriores, el agente está indeciso

# Ejemplo de uso
if __name__ == "__main__":
    agente = AgenteLogico()  # Creamos una instancia del agente

    # El agente observa algunos hechos
    agente.observar('soleado')  # Observa que está soleado
    agente.observar('temperatura_alta')  # Observa que la temperatura es alta

    # El agente decide qué acción tomar
    accion = agente.decidir_accion()  # El agente decide qué acción tomar en base a los hechos observados
    print("Acción a tomar:", accion)  # Imprimimos la acción que el agente decide tomar
