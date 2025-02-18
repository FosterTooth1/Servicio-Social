import ctypes
from ctypes import c_int, c_float, POINTER
import os

# Mapeamos la estructura SimulationResult de C
class SimulationResult(ctypes.Structure):
    _fields_ = [
        ("n", c_int),           # Número de filas (iteraciones)
        ("data", POINTER(c_float))  # Arreglo plano de floats (n*8 elementos)
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

def main():
    # Se asume que la DLL se llama "simulacion.dll" y se encuentra en el mismo directorio que este script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_name = "simulacion.dll"
    lib_path = os.path.join(current_dir, lib_name)
    
    if not os.path.exists(lib_path):
        raise RuntimeError(f"No se encontró la biblioteca en {lib_path}")
    
    sim = Simulator(lib_path)
    resultados = sim.simular(
        delta_t=0.01,
        limite_tiempo=0.2,
        cond_x=0.0,
        cond_y=0.0,
        cond_phi=0.0,
        cond_vl=0.0,
        cond_vr=0.0,
        B=0.20
    )
    
    # Imprimimos cada iteración
    for row in resultados:
        print("t = {:.2f}, x = {:.2f}, y = {:.2f}, phi = {:.2f}, vl = {:.2f}, vr = {:.2f}, u1 = {:.2f}, u2 = {:.2f}"
              .format(*row))

if __name__ == "__main__":
    main()
