#Victor Eduardo Aleman Padilla 21310193
# Definir hechos de la familia
familia = {
    "juan": {"padre": ["maria", "pedro"]},  # Juan es padre de Maria y Pedro
    "ana": {"madre": ["maria", "pedro"]},    # Ana es madre de Maria y Pedro
    "pedro": {"padre": ["carlos", "luis"], "madre": ["maria", "luis"]},  # Pedro es padre de Carlos y Luis, y Maria es madre de ellos
    "maria": {"madre": ["carlos", "luis"]}  # Maria es madre de Carlos y Luis
}

# Definir regla para encontrar ancestros
def ancestro(persona, target):
    if persona == target:  # Si la persona es el objetivo, entonces es su propio ancestro
        return True
    for rel in familia.get(persona, {}).get("padre", []) + familia.get(persona, {}).get("madre", []):
        # Recursivamente buscar en los padres y madres de la persona
        if ancestro(rel, target):
            return True
    return False  # Si no se encontró el ancestro, devolver False

# Consultas de ejemplo
def consultar_hijos_de(padre):
    hijos = familia.get(padre, {}).get("padre", [])  # Obtener los hijos del padre
    print(f"Los hijos de {padre} son: {hijos}")

def consultar_padre_de(hijo):
    for padre in familia:
        if hijo in familia[padre].get("padre", []):  # Buscar al padre de un hijo
            print(f"El padre de {hijo} es: {padre}")
            return
    print(f"No se encontró el padre de {hijo}")  # Si no se encuentra el padre, imprimir un mensaje

def consultar_ancestros_de(persona):
    ancestros = [p for p in familia if ancestro(p, persona) and p != persona]  # Encontrar todos los ancestros de una persona
    print(f"Los ancestros de {persona} son: {ancestros}")

def consultar_hermanos_de(hermano):
    padre_hermano = None
    madre_hermano = None
    # Encontrar al padre del hermano
    for padre, info in familia.items():
        if hermano in info.get("padre", []):
            padre_hermano = padre
            break
    # Encontrar a la madre del hermano
    for madre, info in familia.items():
        if hermano in info.get("madre", []):
            madre_hermano = madre
            break
    # Si tanto el padre como la madre se encuentran, entonces el hermano tiene hermanos
    if padre_hermano and madre_hermano:
        hermanos = [h for h in familia[padre_hermano]["padre"] if h in familia[madre_hermano]["madre"]]
        print(f"Los hermanos de {hermano} son: {hermanos}")
    else:
        print(f"No se encontraron hermanos para {hermano}")  # Si no se encuentran hermanos, imprimir un mensaje

# Ejemplos de consultas
consultar_hijos_de("juan")  # Consultar los hijos de Juan
consultar_padre_de("carlos")  # Consultar el padre de Carlos
consultar_ancestros_de("carlos")  # Consultar los ancestros de Carlos
consultar_hermanos_de("luis")  # Consultar los hermanos de Luis
