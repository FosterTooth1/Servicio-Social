import ctypes
from ctypes import c_int, c_float, POINTER
import os
import matplotlib.pyplot as plt
import math

# Mapeamos la estructura SimulationResult de C
class SimulationResult(ctypes.Structure):
    _fields_ = [
        ("n", c_int),                # Número de iteraciones
        ("data", POINTER(c_float))   # Arreglo plano de floats 
    ]

class Simulator:
    def __init__(self, lib_path):
        # Cargamos la biblioteca
        self.lib = ctypes.CDLL(lib_path)
        # Configuramos el tipo de retorno y los argumentos de la función simular.
        self.lib.simular.restype = POINTER(SimulationResult)
        self.lib.simular.argtypes = [c_float, c_float, c_float, c_float, c_float,
                                     c_float, c_float, c_float, c_float, c_float]
        # Configuramos la función para liberar la memoria
        self.lib.liberar_resultado.argtypes = [POINTER(SimulationResult)]
    
    def simular(self, delta_t, limite_tiempo, cond_x, cond_y, cond_phi,
                cond_vl, cond_vr, B, target_x, target_y):
        res_ptr = self.lib.simular(delta_t, limite_tiempo, cond_x, cond_y, cond_phi,
                                   cond_vl, cond_vr, B, target_x, target_y)
        if not res_ptr:
            raise RuntimeError("Error en la simulación")
        res = res_ptr.contents
        n_iter = res.n
        # Extraemos los datos de la simulación (arreglo plano)
        flat_data = [res.data[i] for i in range(n_iter * 8)]
        # Liberamos la memoria asignada en C
        self.lib.liberar_resultado(res_ptr)
        # Convertimos el arreglo plano en una lista de listas (cada sublista tiene 8 elementos)
        result = [flat_data[i*8:(i+1)*8] for i in range(n_iter)]
        return result

class PointPlotter:
    def __init__(self, points, target):
        self.points = points  # Lista de tuplas (x, y) del robot
        self.target = target  # Tupla (target_x, target_y)
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Trayectoria del Robot y Punto Objetivo")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        
        # Graficamos el punto objetivo de forma permanente
        self.ax.plot(self.target[0], self.target[1], 'go', markersize=10, label="Objetivo")
        
        # Ajustamos los límites de la gráfica según el rango de los puntos y el objetivo
        xs = [p[0] for p in points] + [self.target[0]]
        ys = [p[1] for p in points] + [self.target[1]]
        margin = 1.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        self.ax.legend()
    
    def show(self):
        # Dibuja todos los puntos sin ningún retardo
        for index, p in enumerate(self.points):
            # Dibuja el punto actual y su etiqueta
            self.ax.plot(p[0], p[1], 'ro')
            self.ax.annotate(f"({p[0]:.2f}, {p[1]:.2f})", (p[0], p[1]),
                             textcoords="offset points", xytext=(5, 5))
            # Si no es el primer punto, dibuja la línea entre el anterior y el actual
            if index > 0:
                prev = self.points[index - 1]
                self.ax.plot([prev[0], p[0]], [prev[1], p[1]], 'b-')
        plt.show()

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_name = "simulacion_po.dll"  
    lib_path = os.path.join(current_dir, lib_name)
    
    if not os.path.exists(lib_path):
        raise RuntimeError(f"No se encontró la biblioteca en {lib_path}")
    
    sim = Simulator(lib_path)
    
    # Definimos el punto objetivo
    target = (15.0, 0.0)
    
    # Ejecutamos la simulación
    resultados = sim.simular(
        delta_t=0.01, # paso de tiempo
        limite_tiempo=7,  # tiempo total de simulación
        cond_x=0.0, # Posición inicial x
        cond_y=0.0, # Posición inicial y
        cond_phi=0.0, # Orientación inicial
        cond_vl=0.0, # Velocidad inicial rueda izquierda
        cond_vr=0.0, # Velocidad inicial rueda derecha
        B=0.20, # Distancia entre ruedas
        target_x=target[0], # Posición objetivo x
        target_y=target[1] # Posición objetivo y
    )
    
    # Extraemos de los resultados únicamente los puntos (x, y)
    points = []
    for row in resultados:
        points.append((row[1], row[2]))  # (x, y)

    if not points:
        print("No hay puntos en la simulación.")
        return

    # Iniciamos la representación con matplotlib sin retardo
    plotter = PointPlotter(points, target)
    plotter.show()

if __name__ == "__main__":
    main()
