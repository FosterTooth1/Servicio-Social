import csv
import matplotlib.pyplot as plt
import numpy as np

def de_casteljau(puntos, t):
    """
    Algoritmo de De Casteljau para calcular un punto en la curva de Bézier
    dados los puntos de control y el parámetro t (0 <= t <= 1).
    """
    puntos_temp = puntos.copy()
    while len(puntos_temp) > 1:
        nuevos_puntos = []
        for i in range(len(puntos_temp) - 1):
            x = (1 - t) * puntos_temp[i][0] + t * puntos_temp[i+1][0]
            y = (1 - t) * puntos_temp[i][1] + t * puntos_temp[i+1][1]
            nuevos_puntos.append((x, y))
        puntos_temp = nuevos_puntos
    return puntos_temp[0]

def generar_curva_bezier(puntos_control, num_puntos=100):
    """
    Genera la curva de Bézier evaluando la función de De Casteljau en num_puntos
    distribuidos uniformemente entre 0 y 1.
    """
    t_vals = np.linspace(0, 1, num_puntos)
    curva = [de_casteljau(puntos_control, t) for t in t_vals]
    return curva

def leer_puntos_desde_csv(nombre_archivo):
    """
    Lee los puntos de control desde un archivo CSV y devuelve una lista de tuplas (x, y).
    """
    puntos_control = []
    with open(nombre_archivo, newline='') as csvfile:
        lector = csv.reader(csvfile)
        next(lector)  # Saltar la cabecera
        for fila in lector:
            x, y = map(float, fila)
            puntos_control.append((x, y))
    return puntos_control

def main():
    # Nombre del archivo CSV que contiene los puntos de control
    nombre_archivo = 'pista_escalada.csv'
    
    # Leer los puntos de control desde el archivo CSV
    puntos_control = leer_puntos_desde_csv(nombre_archivo)
    
    # Generar la curva de Bézier
    curva = generar_curva_bezier(puntos_control, num_puntos=100)
    
    # Separar coordenadas para graficar
    curva_x = [p[0] for p in curva]
    curva_y = [p[1] for p in curva]
    control_x = [p[0] for p in puntos_control]
    control_y = [p[1] for p in puntos_control]
    
    # Graficar la curva y la poligonal de control
    plt.figure(figsize=(8,6))
    plt.plot(curva_x, curva_y, label="Curva de Bézier", color="blue")
    plt.plot(control_x, control_y, 'ro--', label="Puntos de control")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Curva de Bézier")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
