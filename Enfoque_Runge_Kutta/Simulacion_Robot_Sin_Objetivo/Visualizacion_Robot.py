import ctypes
from ctypes import c_int, c_float, POINTER
import os
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton

# Mapeamos la estructura SimulationResult de C
class SimulationResult(ctypes.Structure):
    _fields_ = [
        ("n", c_int),                # Número de filas (iteraciones)
        ("data", POINTER(c_float))   # Arreglo plano de floats (n*8 elementos)
    ]

class Simulator:
    def __init__(self, lib_path):
        # Cargamos la DLL
        self.lib = ctypes.CDLL(lib_path)
        # Configuramos el tipo de retorno y los argumentos de la función simular
        self.lib.simular.restype = POINTER(SimulationResult)
        self.lib.simular.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float, c_float]
        # Configuramos la función para liberar la memoria
        self.lib.liberar_resultado.argtypes = [POINTER(SimulationResult)]
    
    def simular(self, delta_t, limite_tiempo, cond_x, cond_y, cond_phi, cond_vl, cond_vr, B):
        res_ptr = self.lib.simular(delta_t, limite_tiempo, cond_x, cond_y, cond_phi, cond_vl, cond_vr, B)
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
    def __init__(self, points):
        self.points = points  # Lista de tuplas (x, y)
        self.index = 0
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Haz clic para mostrar el siguiente punto")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        
        # Ajustamos los límites de la gráfica según el rango de los puntos
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        margin = 1.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        
        # Conectamos el evento de clic con la función on_click
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
    
    def on_click(self, event):
        # Procesamos solo clic izquierdo
        if event.button != MouseButton.LEFT:
            return
        if self.index < len(self.points):
            p = self.points[self.index]
            # Dibuja el punto actual y su etiqueta
            self.ax.plot(p[0], p[1], 'ro')
            self.ax.annotate(f"({p[0]:.2f}, {p[1]:.2f})", (p[0], p[1]),
                             textcoords="offset points", xytext=(5, 5))
            # Si no es el primer punto, dibuja la línea desde el anterior hasta el actual
            if self.index > 0:
                prev = self.points[self.index - 1]
                self.ax.plot([prev[0], p[0]], [prev[1], p[1]], 'b-')
            self.fig.canvas.draw()
            self.index += 1
        else:
            # Cuando se hayan mostrado todos los puntos, se cierra la figura
            plt.close(self.fig)
    
    def show(self):
        plt.show()

def main():
    # Se asume que la DLL se llama "simulacion.dll" y se encuentra en el mismo directorio que este script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_name = "simulacion.dll"
    lib_path = os.path.join(current_dir, lib_name)
    
    if not os.path.exists(lib_path):
        raise RuntimeError(f"No se encontró la biblioteca en {lib_path}")
    
    sim = Simulator(lib_path)
    resultados = sim.simular(
        delta_t=0.01, # Paso de tiempo
        limite_tiempo=600, # Tiempo total de simulación (10 minutos)
        cond_x=0.0, # Condición inicial en x
        cond_y=0.0, # Condición inicial en y
        cond_phi=0.0, # Condición inicial en phi
        cond_vl=0.0, # Condición inicial en vl
        cond_vr=0.0, # Condición inicial en vr
        B=0.20 # Distancia entre llantas
    )
    
    # Extraemos de los resultados únicamente los puntos (x, y) cada 60 segundos.
    points = []
    next_threshold = 60.0
    for row in resultados:
        if row[0] >= next_threshold:
            points.append((row[1], row[2]))  # (x, y)
            next_threshold += 60.0

    if not points:
        print("No hay puntos cada 60 segundos en la simulación.")
        return

    # Iniciamos la representación interactiva con matplotlib
    plotter = PointPlotter(points)
    plotter.show()

if __name__ == "__main__":
    main()
