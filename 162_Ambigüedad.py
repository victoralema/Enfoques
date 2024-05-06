#Victor Eduardo Aleman Padilla 21310193
class AmbiguityDetector:
    def __init__(self):
        pass
        # Inicializador de la clase AmbiguityDetector. No hace nada en este ejemplo.

    def detect_ambiguity(self, sentence):
        # Método para detectar la ambigüedad en una frase dada.
        
        # Lista de frases ambiguas y sus interpretaciones
        ambiguous_sentences = {
            "I saw the man with the telescope.": ["El hombre que vi tenía un telescopio.", "Yo vi al hombre que tenía un telescopio."],
            "Time flies like an arrow.": ["El tiempo vuela como una flecha.", "Mide el tiempo que vuela como una flecha."],
            "Visiting relatives can be boring.": ["Visitar a los familiares puede ser aburrido.", "Los familiares que visitan pueden ser aburridos."]
        }

        # Verificar si la frase está en la lista de frases ambiguas
        if sentence in ambiguous_sentences:
            # Si la frase es ambigua, devuelve las interpretaciones asociadas con esa frase
            return ambiguous_sentences[sentence]
        else:
            # Si la frase no es ambigua, devuelve un mensaje indicando que la frase no es ambigua.
            return ["La frase no es ambigua."]

def main():
    detector = AmbiguityDetector()

    # Frases para analizar
    sentences = [
        "I saw the man with the telescope.",
        "Time flies like an arrow.",
        "Visiting relatives can be boring.",
        "This is a straightforward sentence."
    ]

    # Analizar cada frase y mostrar las interpretaciones si es ambigua
    for sentence in sentences:
        interpretations = detector.detect_ambiguity(sentence)
        print(f"Interpretaciones de '{sentence}': {interpretations}")

if __name__ == "__main__":
    main()
