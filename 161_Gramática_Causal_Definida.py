#Victor Eduardo Aleman Padilla 21310193
class CausalGrammar:
    def __init__(self):
        # Constructor de la clase CausalGrammar, define las reglas de la gramática causal.
        self.rules = {
            "rain": ["wet_ground"],   # Si llueve, el suelo estará mojado.
            "watering": ["wet_ground"],  # Si se riega, el suelo estará mojado.
            "sun": ["dry_ground"],    # Si hay sol, el suelo estará seco.
            "dry_ground": ["cracks"]  # Si el suelo está seco, habrá grietas.
        }

    def infer_effects(self, event):
        # Método para inferir los efectos causales de un evento dado.
        effects = self.rules.get(event, [])  # Obtiene los efectos asociados con el evento, si no hay ninguno, devuelve una lista vacía.
        return effects

def main():
    # Función principal del programa.
    grammar = CausalGrammar()  # Creamos una instancia de la clase CausalGrammar.

    # Ejemplos de eventos para los que queremos inferir efectos.
    events = ["rain", "sun", "watering"]

    # Inferimos y mostramos los efectos causales de cada evento.
    for event in events:
        effects = grammar.infer_effects(event)  # Inferimos los efectos causales del evento actual.
        print(f"The effects of '{event}' are: {effects}")  # Imprimimos los efectos causales.

if __name__ == "__main__":
    main()  # Llamamos a la función main cuando se ejecuta el script directamente.
