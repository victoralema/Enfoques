#Victor Eduardo Aleman Padilla 21310193


import speech_recognition as sr
import random

# Lista de reproducción de música
playlist = {
    "relajante": ["Canción A", "Canción B", "Canción C"],
    "energética": ["Canción X", "Canción Y", "Canción Z"]
}

def reconocimiento_de_habla():
    # Inicializar el reconocedor
    recognizer = sr.Recognizer()
    
    # Utilizar el micrófono como fuente de entrada
    with sr.Microphone() as source:
        print("Di algo sobre el tipo de música que te gustaría escuchar:")
        audio = recognizer.listen(source)
    
    try:
        # Utilizar el reconocedor de Google para convertir el audio en texto
        frase = recognizer.recognize_google(audio, language='es-ES')
        print("Has dicho:", frase)
        # Buscar palabras clave en la frase
        palabras_clave = []
        for palabra in frase.split():
            if palabra.lower() in ("relajante", "energética"):
                palabras_clave.append(palabra.lower())
        # Generar lista de reproducción basada en las palabras clave
        if palabras_clave:
            print("\nGenerando lista de reproducción...")
            for palabra_clave in palabras_clave:
                if palabra_clave in playlist:
                    print(f"\nLista de reproducción {palabra_clave.capitalize()}:")
                    for cancion in playlist[palabra_clave]:
                        print(cancion)
                else:
                    print(f"No hay lista de reproducción disponible para {palabra_clave}")
        else:
            print("\nNo se encontraron palabras clave relacionadas con música en tu frase.")
    except sr.UnknownValueError:
        print("No se pudo entender el audio")
    except sr.RequestError as e:
        print("Error al solicitar resultados desde el servicio de Google: {0}".format(e))

if __name__ == "__main__":
    reconocimiento_de_habla()
