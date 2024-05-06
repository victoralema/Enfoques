#Victor Eduardo Aleman Padilla 21310193
import cv2

# Cargamos la imagen en escala de grises
imagen = cv2.imread('paisaje.jpg', cv2.IMREAD_GRAYSCALE)

# Verificamos si la imagen se cargó correctamente y tiene dimensiones válidas
if imagen is not None and imagen.shape[0] > 0 and imagen.shape[1] > 0:
    # Aplicamos el algoritmo de Canny para detección de bordes
    bordes = cv2.Canny(imagen, 100, 200)

    # Mostramos la imagen original y la detección de bordes
    cv2.imshow('Imagen Original', imagen)
    cv2.imshow('Detección de Bordes (Canny)', bordes)

    # Esperamos a que se presione cualquier tecla y luego cerramos las ventanas
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Error: No se pudo cargar la imagen o tiene dimensiones no válidas.")

