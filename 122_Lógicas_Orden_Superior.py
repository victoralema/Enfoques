#Victor Eduardo Aleman Padilla 21310193
from sympy import symbols, Implies  # Importamos símbolos y la función de implicación de Sympy

# Definimos las variables simbólicas p y q
p, q = symbols('p q')

# Definimos las premisas y la conclusión como una lista de tuplas, donde el primer elemento es la expresión lógica y el segundo indica si es verdadera o no
premisas = [(p, True), (Implies(p, q), True)]
conclusion = q  # Definimos la conclusión como q

# Función para aplicar modus ponens
def modus_ponens(premisas, conclusion):
    for premisa in premisas:  # Iteramos sobre cada premisa en la lista de premisas
        if not premisa[1]:  # Verificamos si la premisa no es verdadera
            return "No se puede aplicar el modus ponens."  # Si alguna premisa no es verdadera, no podemos aplicar modus ponens
    
    # Verificamos si la conclusión sigue de las premisas mediante modus ponens
    for premisa in premisas:  # Iteramos sobre cada premisa en la lista de premisas
        if premisa[0] == conclusion:  # Si la premisa implica la conclusión
            return "Modus Ponens se aplica correctamente."  # Se puede aplicar modus ponens correctamente
    
    return "La conclusión no sigue de las premisas mediante Modus Ponens."  # Si no encontramos una premisa que implique la conclusión, no se puede aplicar modus ponens

# Aplicar el modus ponens
resultado = modus_ponens(premisas, conclusion)  # Llamamos a la función modus_ponens con las premisas y la conclusión
print(resultado)  # Imprimimos el resultado de la aplicación de modus ponens
