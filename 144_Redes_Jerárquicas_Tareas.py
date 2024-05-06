#Victor Eduardo Aleman Padilla 21310193
# Importar la biblioteca networkx para trabajar con grafos
import networkx as nx

# Definir una clase para el planificador de tareas
class TaskScheduler:
    # Método de inicialización para crear un grafo dirigido vacío
    def __init__(self):
        self.graph = nx.DiGraph()  # Crear un grafo dirigido vacío

    # Método para agregar una tarea y sus dependencias al grafo
    def add_task(self, task, dependencies=None):
        self.graph.add_node(task)  # Agregar el nodo de la tarea al grafo
        if dependencies:  # Si hay dependencias especificadas
            for dependency in dependencies:  # Iterar sobre las dependencias
                self.graph.add_edge(dependency, task)  # Agregar un arco de la dependencia a la tarea

    # Método para planificar las tareas en base a las dependencias
    def schedule_tasks(self):
        # Utilizar el algoritmo topológico para obtener un ordenamiento topológico de las tareas
        return nx.topological_sort(self.graph)

# Función principal
if __name__ == "__main__":
    # Crear una instancia del planificador de tareas
    scheduler = TaskScheduler()

    # Agregar tareas y sus dependencias
    scheduler.add_task("Tarea A")  # Agregar la tarea A sin dependencias
    scheduler.add_task("Tarea B", dependencies=["Tarea A"])  # Agregar la tarea B con dependencia en la tarea A
    scheduler.add_task("Tarea C", dependencies=["Tarea B", "Tarea A"])  # Agregar la tarea C con dependencias en las tareas B y A
    scheduler.add_task("Tarea D", dependencies=["Tarea C"])  # Agregar la tarea D con dependencia en la tarea C
    scheduler.add_task("Tarea E", dependencies=["Tarea C"])  # Agregar la tarea E con dependencia en la tarea C

    # Planificar y mostrar el orden de las tareas
    task_order = scheduler.schedule_tasks()  # Obtener el orden de las tareas planificadas
    print("Orden de tareas:")  # Imprimir un mensaje
    for task in task_order:  # Iterar sobre las tareas en el orden planificado
        print(task)  # Imprimir el nombre de la tarea
